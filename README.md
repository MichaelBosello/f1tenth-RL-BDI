# BDI-RL Framework Proof of Concept
This project is a PoC of the framework presented in 

***From Programming Agents to Educating Agents – A Jason-based Framework for Integrating Learning in the Development of Cognitive Agents***

Paper avaliable at: http://cgi.csc.liv.ac.uk/~lad/emas2019/accepted/EMAS2019_paper_33.pdf

Slides of the presentation at EMAS: https://www.slideshare.net/MichaelBosello/emas-2019-from-programming-agents-to-educating-agents

This is an integration of BDI agents and Reinforcement Learning.
It is based on [Jason](http://jason.sourceforge.net/wp/) (Actually, it is a [JaCaMo](http://jacamo.sourceforge.net/) project).

The basic idea is that a developer could write some plans and let the agent itself learn other plans and use them in a seamless way. This is not only for a specific ad hoc problem but as a general feature of the agent platform.

In short, the aim of the framework is to enable the developer to define the learning components with high-level abstractions as the BDI ones are. Then, these informations injeced by the developer are used by the agent to learn itself how to fulfill some tasks. 

The work of the developer moves from write plans to define a learning phase.


# Quick start
Build the project

	./gradlew build

Stop previous istances

	./gradlew --stop

Run the python agent server

	./gradlew runPythonAgent


Run the python environment server

	./gradlew runPythonEnv

Run the agent system:

	./gradlew run

## dependencies
Install Java

	sudo apt-get install openjdk-8-jdk

	pip3 install flask flask-jsonpify flask-restful

# Configuration
## Algorithm Parameters
You can change the parameters with the agent beliefs.
