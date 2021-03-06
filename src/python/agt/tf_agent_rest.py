from flask import Flask, jsonify, request
from flask_restful import Resource, Api
import json
import numpy as np

from car_driver.rl_car_driver import DqnAgent

app = Flask(__name__)
app.debug = False
app.testing = False
api = Api(app)

import logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

agents = {}

class Env(Resource):
    def post(self, agent_id):
        if not agent_id in agents:
            json_data = request.get_json(force=True)
            print("##################################")
            print("""a_shape {}, a_type {}, a_min {}, a_max {}, o_shape {}, o_type {}, init_state {},
            agent_type {}, parameters {}""".format(json_data['a_shape'], json_data['a_type'],
            json_data['a_min'], json_data['a_max'], json_data['o_shape'], json_data['o_type'],
            json_data['init_state'], json_data['agent_type'], json_data['parameters']))
            print("##################################")
            if json_data['agent_type'] == "dqn":
                agent = DqnAgent(json_data['a_max'][0] + 1, json_data['o_shape'][0], json_data['parameters'], agent_id)
            agents[agent_id] = agent

class Action(Resource):
    def post(self, agent_id, action_type):
        json_data = request.get_json(force=True)
        #print("##################################")
        #print(json_data)
        if action_type == "next_train_action":
            action = agents[agent_id].epoch_step(np.array(json_data['state'], dtype=json_data['state_type']),
                                      json_data['reward'], json_data['is_terminal'])
        if action_type == "next_best_action":
            action = agents[agent_id].inference(np.array(json_data['state'], dtype=json_data['state_type']),
                                          json_data['reward'], json_data['is_terminal'])
        result = {'action': [int(action)]}
        #print("##################################")
        #print(result)
        return jsonify(result)

api.add_resource(Env, '/agent/<string:agent_id>')
api.add_resource(Action, '/agent/<string:agent_id>/<string:action_type>')

if __name__ == '__main__':
     app.run(port='5002')