from flask import Flask, render_template, Response, request
import time



app = Flask(__name__)



@app.route("/", methods=['GET', 'POST'])
def index():
	return render_template('tempIndex.html')

if __name__ == "__main__":
    app.run()