from __future__ import annotations

from pathlib import Path
import re

import dash
from dash import dcc, html
import dash_bootstrap_components as dbc


BASE_DIR = Path(__file__).resolve().parent
DAY_FILES = {
    "Day 1": BASE_DIR / "day_1_final.md",
    "Day 2": BASE_DIR / "day_2_final.md",
    "Day 3": BASE_DIR / "day_3_final.md",
    "Day 4": BASE_DIR / "day_4_final.md",
}


def load_md(path: Path) -> str:
    try:
        content = path.read_text(encoding="utf-8")
        # Render direct MP4 links as embedded videos and make URLs clickable.
        lines = []
        mp4_re = re.compile(r"(https?://\\S+?\\.mp4)")
        url_re = re.compile(r"(https?://\\S+)")
        for line in content.splitlines():
            matches = mp4_re.findall(line)
            if not matches:
                # Autolink any URLs in the line.
                def _linkify(match: re.Match) -> str:
                    url = match.group(1)
                    return f"[{url}]({url})"

                lines.append(url_re.sub(_linkify, line))
                continue
            # Keep any non-URL text, then add video embeds.
            stripped_text = mp4_re.sub("", line).strip()
            if stripped_text:
                # Autolink any URLs in the remaining text.
                def _linkify(match: re.Match) -> str:
                    url = match.group(1)
                    return f"[{url}]({url})"

                lines.append(url_re.sub(_linkify, stripped_text))
            for url in matches:
                lines.append(f'<video controls src="{url}"></video>')
                lines.append(f"[Video source]({url})")
        return "\n".join(lines)
    except FileNotFoundError:
        return f"# Missing file\n\nCould not find `{path.name}`."


app = dash.Dash(__name__, external_stylesheets=[dbc.themes.FLATLY])
server = app.server


tabs = []
for day, path in DAY_FILES.items():
    tabs.append(
        dbc.Tab(
            label=day,
            tab_id=day.lower().replace(" ", "-"),
            children=dbc.Card(
                dbc.CardBody(
                    dcc.Markdown(
                        load_md(path),
                        link_target="_blank",
                        dangerously_allow_html=True,
                    ),
                ),
                class_name="shadow-sm",
            ),
        )
    )


app.layout = dbc.Container(
    [
        html.H1("Unitree G1 Academy - Course Days", className="mt-3"),
        html.P(
            "Browse each day of the course. Content is loaded from the local markdown files.",
            className="text-muted",
        ),
        dbc.Tabs(tabs, active_tab="day-1"),
    ],
    fluid=True,
)


if __name__ == "__main__":
    app.run(debug=True, host="0.0.0.0", port=8050)
