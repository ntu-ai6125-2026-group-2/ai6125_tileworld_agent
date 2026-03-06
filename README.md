# AI6125-Tileworld Agent

Building intelligent agents for the Tileworld environment.

This repository contains the Java source code for a Tileworld simulation framework, including:
- Environment and entities (`environment/`)
- Agent base classes and utilities (`agent/`)
- Path planning utilities (`planners/`)
- Runtime parameters and entry points (`TileworldMain`, `Parameters`, GUI)

## Goal

The goal is to implement an agent that maximizes total rewards in the Tileworld setting.

## Agent Implementation Modules

Three modules to be implemented:

1. Planning module
	- After sensing/communication, the agent's memory is updated.
	- The agent then plans its next action accordingly.

2. Memory module
	- Agents store environmental information in memory.
	- The default module is `TWAgentWorkingMemory`.

3. Communication module
	- At every step, each agent can broadcast a message encoded by `Message`.