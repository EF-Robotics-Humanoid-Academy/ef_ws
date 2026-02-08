#!/usr/bin/env python3
import os
import re
import time
import json
import queue
import threading
import rclpy
import speech_recognition as sr

from openai import OpenAI
from rclpy.node import Node
from g1_llm.actions import G1ArmActions
from g1_llm.cmd_vel import BaseCmdVel
from g1_llm.image import ImageSubscriber, ImageExplainer
from g1_interface.srv import G1Modes

# --- Config -------------------------------------------------------------------
CHUNK_MS = 20

# LLM provider/model
PROVIDER = os.getenv("PROVIDER", "openai").lower()
MODEL_NAME = os.getenv("OPENAI_MODEL", "gpt-4o-mini")

# Streaming color + wake word gate
STREAM_COLOR = "yellow"
WAKE_WORD = "danny"
WAKE_WINDOW_S = float(3)

# --- ANSI color helper --------------------------------------------------------
def colorize(text: str, color: str) -> str:
    colors = {
        "red": "\033[31m", "green": "\033[32m",
        "yellow": "\033[33m", "blue": "\033[34m",
        "magenta": "\033[35m", "cyan": "\033[36m",
        "reset": "\033[0m"
    }
    return f"{colors.get(color,'')}{text}{colors['reset']}"

# --- LLM tools ---------------------------------------------------------------
tools = [
    # Arms
    {
        "type": "function",
        "function": {
            "name": "dual_wave",
            "description": "Make both arms perform the predefined dual-wave trajectory.",
            "parameters": {"type": "object", "properties": {}}
        },
    },
    {
        "type": "function",
        "function": {
            "name": "arms_down",
            "description": "Return both arms to the down/rest pose.",
            "parameters": {"type": "object", "properties": {}}
        },
    },
    {
        "type": "function",
        "function": {
            "name": "cancel_motion",
            "description": "Cancel the current FollowJointTrajectory goal for the arms.",
            "parameters": {"type": "object", "properties": {}}
        },
    },
    # CMD Vel
    {
        "type": "function",
        "function": {
            "name": "drive_base",
            "description": "Drives the robot using cmd_vel. Positive x forward, negative x backward, positive y to left moves left, \
                negative y to right moves right, positive wz rotates left as well as looks left and negative wz rotates right as well as looks right.\
                Minimum velocity is 0.2 m/s and minimum angular velocity is 0.5 rad/s.",
            "parameters": {
                "type": "object",
                "properties": {
                    "vx": {"type": "number", "description": "Linear X m/s (forward+ or backward-)"},
                    "vy": {"type": "number", "description": "Linear Y m/s (left+ or right-). "},
                    "wz": {"type": "number", "description": "Angular Z rad/s (+ve rotates left or looks left, -ve rotates right or looks right)."},
                    "duration": {"type": "number", "description": "Seconds to apply (>= 0)"}
                },
                "required": ["vx", "vy", "wz", "duration"]
            }
        },
    },
    {
        "type": "function",
        "function": {
            "name": "stop_base",
            "description": "Immediately stop the base by publishing a zero Twist.",
            "parameters": {"type": "object", "properties": {}}
        },
    },
    # LEDs
    {
        "type": "function",
        "function": {
            "name": "change_status_color",
            "description": "Change the robot's LED/status color using an HTML color name or hex value.",
            "parameters": {
                "type": "object",
                "properties": {"color": {"type": "string"}},
                "required": ["color"],
            },
        },
    },
    # Image
    {
        "type": "function",
        "function": {
            "name": "describe_view",
            "description": "Describe the latest camera image in plain language.",
            "parameters": {
                "type": "object",
                "properties": {
                    "focus": {
                        "type": "string",
                        "description": "Optional focus or instruction, e.g., 'read the whiteboard', 'what objects are on the table'"
                    }
                }
            }
        },
    },
]

# --- Provider/client factory ---------------------------------------------------
def get_openai_compatible_client():
    if PROVIDER == "openrouter":
        base_url = os.getenv("OPENROUTER_BASE_URL",
                             "https://openrouter.ai/api/v1")
        api_key = os.getenv("OPENROUTER_API_KEY")
        if not api_key:
            raise RuntimeError(
                "OPENROUTER_API_KEY is required for provider=openrouter")
        return OpenAI(base_url=base_url, api_key=api_key)
    return OpenAI()

# --- LLM streaming ------------------------------------------------------------
def stream_chat_text(prompt: str, logger, model=MODEL_NAME, context=None, max_tokens=150):
    client = get_openai_compatible_client()

    full = []
    tool_calls = {}
    messages = []
    if context:
        messages.extend(context)
    messages.append({"role": "user", "content": prompt})

    stream = client.chat.completions.create(
        model=model,
        messages=messages,
        tools=tools,
        max_tokens=max_tokens,
        stream=True,
    )

    for event in stream:
        delta = event.choices[0].delta

        # text streaming
        if delta.content:
            token = delta.content
            full.append(token)
            logger.info(colorize(token, STREAM_COLOR))

        # function call streaming
        if delta.tool_calls:
            for call in delta.tool_calls:
                idx = call.index
                if idx not in tool_calls:
                    tool_calls[idx] = {"name": None, "args": ""}
                if call.function and call.function.name:
                    tool_calls[idx]["name"] = call.function.name
                if call.function and call.function.arguments:
                    tool_calls[idx]["args"] += call.function.arguments

    calls = []
    for tc in tool_calls.values():
        calls.append((tc["name"], tc["args"]))
    return "".join(full), calls

# --- TTS cleanup --------------------------------------------------------------
def clean_for_speech(text: str) -> str:
    cleaned = re.sub(r"[^a-zA-Z0-9.,?! ]+", " ", text)
    cleaned = re.sub(r"\s+", " ", cleaned).strip()
    return cleaned


# --- Audio / Speech recognition ------------------------------------------------
class SpeechRecognitionBackground:
    """
    Mic index 0, continuous background capture.
    - Background callback only enqueues audio (non-blocking)
    - A worker thread runs recognition; queue bounded to avoid pile-ups
    - Wake gating remains in your _tick() as-is
    """

    def __init__(self, device_index=0):
        self.backend = os.getenv(
            "SR_BACKEND", "google").lower()   # 'google' | 'sphinx'
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone(device_index=device_index)

        # Queues
        # incoming phrases from callback
        self._audio_q = queue.Queue(maxsize=4)
        # recognized texts for the node
        self._results_q = queue.Queue(maxsize=16)

        # Tunables (env overrides)
        self.adjust_duration = float(os.getenv("SR_ADJUST_S", "0.5"))
        self.phrase_time_limit = float(os.getenv("SR_PHRASE_S", "4"))
        self.pause_threshold = float(
            os.getenv("SR_PAUSE_S", "0.6"))          # default 0.8
        self.non_speaking_duration = float(
            os.getenv("SR_NONSPEAK_S", "0.4"))  # default 0.5
        # pin post-calibration
        self.energy_threshold = int(os.getenv("SR_ENERGY", "250"))

        self._stopper = None
        self._worker_stop = threading.Event()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)

    def start(self):
        # One-time calibration, then pin levels to avoid drift after TTS playback
        with self.microphone as source:
            self.recognizer.dynamic_energy_threshold = True
            self.recognizer.adjust_for_ambient_noise(
                source, duration=self.adjust_duration)
            # Pin and disable dynamic drift
            self.recognizer.energy_threshold = self.energy_threshold or self.recognizer.energy_threshold
            self.recognizer.dynamic_energy_threshold = False
            # Segmentation tuning
            self.recognizer.pause_threshold = self.pause_threshold
            self.recognizer.non_speaking_duration = self.non_speaking_duration

        # Start background capture (spawns its own reader thread)
        self._stopper = self.recognizer.listen_in_background(
            self.microphone, self._callback, phrase_time_limit=self.phrase_time_limit
        )

        # Start recognition worker
        self._worker.start()

    def stop(self):
        if self._stopper:
            try:
                self._stopper(wait_for_stop=False)
            except Exception:
                pass
            self._stopper = None
        self._worker_stop.set()
        try:
            self._worker.join(timeout=0.5)
        except Exception:
            pass

    # ---------- background callback (fast, never blocks) ----------
    def _callback(self, recognizer: sr.Recognizer, audio: sr.AudioData):
        try:
            self._audio_q.put_nowait(audio)
        except queue.Full:
            # Drop oldest to keep things moving
            try:
                _ = self._audio_q.get_nowait()
                self._audio_q.put_nowait(audio)
            except queue.Empty:
                pass

    # ---------- recognition worker (can block, not on the capture thread) ----------
    def _worker_loop(self):
        while not self._worker_stop.is_set():
            try:
                audio = self._audio_q.get(timeout=0.1)
            except queue.Empty:
                continue
            text = ""
            try:
                if self.backend == "sphinx":
                    text = self.recognizer.recognize_sphinx(
                        audio)       # offline
                else:
                    text = self.recognizer.recognize_google(
                        audio)       # cloud
                text = (text or "").strip()
            except sr.UnknownValueError:
                text = ""
            except sr.RequestError as e:
                text = f"[SpeechRecognition error: {e}]"
            finally:
                if text:
                    try:
                        self._results_q.put_nowait(text)
                    except queue.Full:
                        # Prefer the newest result
                        try:
                            _ = self._results_q.get_nowait()
                            self._results_q.put_nowait(text)
                        except queue.Empty:
                            pass

    def poll_result(self):
        try:
            return True, self._results_q.get_nowait()
        except queue.Empty:
            return False, ""

    # Optional: on-demand recalibration (e.g., map it to a voice/keyboard command)
    def recalibrate(self, seconds=0.5):
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=seconds)
            self.recognizer.energy_threshold = max(
                150, int(self.recognizer.energy_threshold))

# --- Node ---------------------------------------------------------------------


class G1AssistantNode(Node):
    def __init__(self):
        super().__init__("g1_always_chatgpt")

        # Start background speech recognition on mic index 0
        self.speech_recognition = SpeechRecognitionBackground(device_index=0)
        self.speech_recognition.start()

        self.timer = self.create_timer(CHUNK_MS/1000.0, self._tick)

        # Arms/action wrapper + ROS2 services (unchanged)
        self.arms = G1ArmActions(self)
        self.base = BaseCmdVel(self)
        
        # ---- Vision (image subscribe + explainer) ----
        self.image_sub = ImageSubscriber(self, topic='d435/image_raw')
        self.vision = ImageExplainer(self, self.image_sub, get_openai_compatible_client(
        ), model=os.getenv("OPENAI_VISION_MODEL", MODEL_NAME))

        self.audio_cli = self.create_client(G1Modes, "hardware/audio")
        self.status_cli = self.create_client(G1Modes, "hardware/led")

        self.history = []
        self.max_history = 5
        self._awake_until = 0.0

        self.get_logger().info(
            colorize(
                f"Always listening → Provider={PROVIDER} Model={MODEL_NAME}", "green")
        )
        self.get_logger().info(
            colorize(
                f"Wake word='{WAKE_WORD}', window={WAKE_WINDOW_S}s, stream color={STREAM_COLOR}", "blue")
        )

    def _tick(self):
        # Poll for a final transcript from background listener
        is_final, text = self.speech_recognition.poll_result()
        if not is_final or not text:
            return

        t_stt_done = time.time()
        raw_text = text.strip()
        low = raw_text.lower()
        now = t_stt_done

        # Wake gating
        if WAKE_WORD in low:
            self._awake_until = now + WAKE_WINDOW_S
            pat = rf"^\s*{re.escape(WAKE_WORD)}\s*[:,\-]?\s*"
            user_cmd = re.sub(pat, "", raw_text, flags=re.IGNORECASE)
        else:
            if now > self._awake_until:
                self.get_logger().info(
                    colorize(f"Ignored (no wake word): {raw_text}", "magenta"))
                return
            user_cmd = raw_text

        self.get_logger().info(colorize(f"User: {user_cmd}", "cyan"))
        self.robot_status('purple')
        self._speak_async("Thinking")

        # 1) Build context
        context = [
            {"role": "system", "content": (
                "You are Danny, a compact humanoid robot for research and education. "
                "Keep conversations very brief. "
            )}
        ]
        context.extend(self.history)
        context.append({"role": "user", "content": user_cmd})

        # 2) Call LLM
        t_llm_start = time.time()
        full, tool_calls = stream_chat_text(
            user_cmd, self.get_logger(), model=MODEL_NAME, context=context, max_tokens=50
        )
        t_llm_end = time.time()
        cleaned = clean_for_speech(full)
        self.get_logger().info(colorize(f"Response: {cleaned}", "green"))

        # 3) Speak
        t_tts_start = time.time()
        try:
            req = G1Modes.Request()
            req.request_data = f"volume=100;speak= {cleaned}"
            self.audio_cli.call_async(req)
        except Exception as e:
            self.get_logger().error(f"TTS error: {e}")

        # Print timings
        self.get_logger().info(colorize(
            f"TIMING: STT → LLM start {t_llm_start - t_stt_done:.3f}s | "
            f"LLM proc {t_llm_end - t_llm_start:.3f}s | "
            f"LLM → TTS call {t_tts_start - t_llm_end:.3f}s",
            "yellow"
        ))

        # 4) Execute tools (with short confirmations)
        for name, args in tool_calls:
            try:
                parsed = json.loads(args) if args else {}
            except Exception as e:
                self.get_logger().error(
                    f"Failed to parse tool args for {name}: {e}")
                parsed = {}

            try:
                if name == "dual_wave":
                    self.arms.send_dual_wave()
                elif name == "arms_down":
                    self.arms.send_down()
                elif name == "cancel_motion":
                    self.arms.cancel()
                elif name == "drive_base":
                    vx = float(parsed.get("vx", 0.0))
                    vy = float(parsed.get("vy", 0.0))
                    wz = float(parsed.get("wz", 0.0))
                    duration = float(parsed.get("duration", 0.0))
                    self.base.drive(vx, vy, wz, duration)
                    self._speak_async(
                        f"Walking for {duration:.1f} seconds")
                elif name == "stop_base":
                    self.base.stop()
                elif name == "change_status_color":
                    color = parsed.get("color", "white")
                    self.robot_status(color)
                    self._speak_async(
                        f"Changing L E D status color to {color}")
                elif name == "describe_view":
                    focus = parsed.get("focus", "") if isinstance(
                        parsed, dict) else ""
                    desc = self.vision.describe(focus)
                    cleaned_desc = clean_for_speech(desc)
                    self._speak_async(f"{cleaned_desc}")
                    self.get_logger().info(
                        colorize(f"Vision: {cleaned_desc}", "green"))
                else:
                    self.get_logger().warn(f"Unknown tool: {name}")
            except Exception as e:
                self.get_logger().error(
                    f"Tool execution error for {name}: {e}")

        # 5) History
        self.history.append({"role": "user", "content": user_cmd})
        self.history.append({"role": "assistant", "content": cleaned})
        if len(self.history) > self.max_history * 2:
            self.history = self.history[-self.max_history*2:]
        
        self.robot_status('green')

    # --- Utilities -------------------------------------------------------------
    def _speak_async(self, text: str):
        try:
            self.get_logger().info(colorize(f"Speaking: {text}", "magenta"))
            req = G1Modes.Request()
            req.request_data = f"volume=100;speak= {text}"
            self.audio_cli.call_async(req)
        except Exception as e:
            self.get_logger().error(f"TTS error: {e}")

    def robot_status(self, html_color: str):
        self.get_logger().info(f"robot_status → color={html_color}")
        req = G1Modes.Request()
        req.request_data = f'color={html_color};brightness=100'
        self.status_cli.call_async(req)

    def colorize(self, s: str, c: str) -> str:
        return colorize(s, c)

# --- Main ---------------------------------------------------------------------
def main():
    rclpy.init()
    node = G1AssistantNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.speech_recognition.stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
