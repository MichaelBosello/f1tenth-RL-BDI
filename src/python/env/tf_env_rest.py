from flask import Flask, jsonify, request
from flask_restful import Resource, Api
import json

app = Flask(__name__)
app.debug = False
app.testing = False
api = Api(app)

import logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

from carenv.car_env import CarEnv

CarEnv.run_node()
envs = {}

class Env(Resource):
    def post(self, id):
        if not id in envs:
            json_data = request.get_json(force=True)
            print("##################################")
            print(json_data)
            env = CarEnv(json_data['parameters'])
            envs[id] = env

        result = {'state': envs[id].state,
                  'reward' : envs[id].reward,
                  'terminal' : envs[id].is_terminal}
        print("##################################")
        #print('starting state', result)
        return jsonify(result)

class Action(Resource):
    def post(self, id, action):
        json_data = request.get_json(force=True)
        #print("##################################")
        #print(json_data)
        reward, state, is_terminal = envs[id].step(int(action))
        result = {'state': state,
                  'reward' : reward,
                  'terminal' : is_terminal}
        #print("##################################")
        #print(result)
        return jsonify(result)

api.add_resource(Env, '/env/<string:id>')
api.add_resource(Action, '/env/<string:id>/<string:action>')

if __name__ == '__main__':
     app.run(port='5003')