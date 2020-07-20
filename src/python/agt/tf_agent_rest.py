from flask import Flask, jsonify, request
from flask_restful import Resource, Api
import json

from rl_car_driver import DqnAgent

app = Flask(__name__)
app.debug = False
app.testing = False
api = Api(app)

import logging
log = logging.getLogger('logger')
log.setLevel(logging.ERROR)

agents = {}

class Env(Resource):
    def post(self, id):
        if not id in agents:
            json_data = request.get_json(force=True)
            print("##################################")
            print(json_data)
            if json_data['agent_type'] == "dqn":
                agent = DqnAgent(
                    json_data['a_shape'], json_data['o_shape'],
                    json_data['init_state'], json_data['parameters']
                )
            agents[id] = agent

class Action(Resource):
    def post(self, id, action_type):
        json_data = request.get_json(force=True)
        #print("##################################")
        #print(json_data)
        if action_type == "next_train_action":
            action = agents[id].get_train_action()
        if action_type == "next_best_action":
            action = agents[id].get_eval_action()

        agents[id].update(np.array(json_data['state'], dtype=json_data['state_type']),
            json_data['reward'], json_data['is_terminal'], action_step)

        result = {'action': action}
        #print("##################################")
        #print(result)
        return jsonify(result)

api.add_resource(Env, '/agent/<string:id>')
api.add_resource(Action, '/agent/<string:id>/<string:action_type>')

if __name__ == '__main__':
     app.run(port='5002')