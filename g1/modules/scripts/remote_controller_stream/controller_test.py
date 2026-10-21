"""Print raw joystick/controller input for debugging."""

import time

import pygame


def main() -> None:
    pygame.init()
    pygame.joystick.init()
    pygame.event.set_allowed(None)
    pygame.event.set_allowed(
        [
            pygame.JOYBUTTONDOWN,
            pygame.JOYBUTTONUP,
            pygame.JOYAXISMOTION,
            pygame.JOYHATMOTION,
            pygame.CONTROLLERBUTTONDOWN,
            pygame.CONTROLLERBUTTONUP,
            pygame.CONTROLLERAXISMOTION,
        ]
    )

    if pygame.joystick.get_count() == 0:
        print("No controller detected.")
        return

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print("Controller:", joystick.get_name())
    print("Axes:", joystick.get_numaxes())
    print("Buttons:", joystick.get_numbuttons())
    print("Hats/D-pads:", joystick.get_numhats())
    print("Press Ctrl+C to stop.\n")

    try:
        while True:
            events = pygame.event.get()

            for event in events:
                if event.type in (
                    pygame.JOYBUTTONDOWN,
                    pygame.JOYBUTTONUP,
                    pygame.CONTROLLERBUTTONDOWN,
                    pygame.CONTROLLERBUTTONUP,
                ):
                    print("button event:", event, flush=True)
                elif event.type == pygame.JOYHATMOTION:
                    print("hat event:", event, flush=True)

            axes = [
                round(joystick.get_axis(index), 3)
                for index in range(joystick.get_numaxes())
            ]
            buttons = [
                bool(joystick.get_button(index))
                for index in range(joystick.get_numbuttons())
            ]
            hats = [
                joystick.get_hat(index)
                for index in range(joystick.get_numhats())
            ]

            print(
                "axes:", axes,
                "buttons:", buttons,
                "hats:", hats,
                flush=True,
            )
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        pygame.quit()


if __name__ == "__main__":
    main()
