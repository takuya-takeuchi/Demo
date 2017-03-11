from flask import Flask
from flask_restful import Resource, Api
import requests
from datetime import datetime as dt

app = Flask(__name__)
api = Api(app)

class DateTimeServiceClient:

    def _send(self, datetime):
        s = requests.session()

        tstr = datetime.strftime("%Y/%m/%d %H:%M:%S.") + "%03d" % (datetime.microsecond // 1000)
        params = {
            'dateTime': tstr,
        }
        headers = {'content-type': 'application/json'}
        r =  s.post('http://localhost:5002/DateTimeService/Send', params=params, headers=headers)

class HelloWorld(Resource):
    def get(self):
        dtsc = DateTimeServiceClient();
        tdatetime = dt.now()
        dtsc._send(tdatetime);
        return {'hello': 'world'}

api.add_resource(HelloWorld, '/')

if __name__ == '__main__':
    app.run(debug=True)