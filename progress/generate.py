import yaml
import json
import requests
from datetime import date
from jinja2 import Template


class Templates:
    __slots__ = ["html", "js"]


templates = Templates()

vue = requests.get("https://unpkg.com/vue@2.7.16/dist/vue.min.js").text


def update_content() -> None:
    with open("template.html") as file:
        templates.html = Template(file.read())

    with open("template.js") as file:
        templates.js = Template(file.read())

    with open("content.yml") as file:
        global content
        content = {k.lower(): v for k, v in yaml.safe_load(file).items()}

    with open("style.css") as file:
        global style
        style = file.read()


def generate_html(filename: str) -> str:
    with open(filename) as file:
        update_content()
        data = json.dumps(yaml.safe_load(file))
        year = date.today().year
        js = templates.js.render(data=data, vue=vue)
        return templates.html.render(date=year, style=style, js=js, **content)
