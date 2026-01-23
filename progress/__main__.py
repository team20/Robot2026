import sys
from generate import generate_html
from flask import Flask


def main() -> None:
    filename = next(arg for arg in sys.argv if arg.endswith(".yml"))
    if "--interactive" in sys.argv:
        app = Flask(__name__)

        @app.route("/")
        def index() -> str:
            return generate_html(filename)

        app.run(debug=True)
    else:
        with open("index.html", "w") as file:
            file.write(generate_html(filename))


if __name__ == "__main__":
    main()
