---
layout: post
title: Behavior Trees as an alternative to FSM and Heirarchical FSM
date: 2023-06-13 09:47:00-0400
description: Description and contrasting Properties of FSMs and HFSMs 
tags: Task Planning
categories: Control Motion Planning Task Planning
bibliography: 2022-08-23-Search-and-Rescue-using-multiple-surface-vehicles.bib
---

### Introduction 
Programming robots is a tedious task and there are many moving parts that require to coordinate with each other. There are many software frameworks and best practices that are used to implement the execution of all the individual modules and interface with each other based on some conditionals. Let's consider an example to elaborate this statement, given we have a simple robot with a depth sensor and two functionalities are implemented i) Detect an obtacle ii) Go left or right based on the free space. Using this it has an obstacle avoidance functionality, I should also point out that there are all the system I/O calls to the hardware that have been written to formulate this solution. A simple flow chart can depict the entire functionality while abstracting out some part of the code. 

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/VictorSierra.png" class="Victor-Sierra Search" zoomable=true %}
    </div>
</div>
<div class="caption">
    Fig. 1 Victor-Sierra Search with Datum information.
</div>

The flow chart shows how we switch while making decisions based on previous inputs. It should be noted here that many people who are ROS users might be thinking that it is implemented as a multi-threaded system, thus an event diagram is a better representation perhaps like a distributed system with RPC calls, agreed but the nodes downstream still represent a sequential execution in terms of information/dataflow thus this representation is considered for visualisation. Comments can be made that this means the system is good enough to be represented by a non-parallelised flow chart, though a parallel system would be a better representation to plot and analyse the same. I'll make more comments about parallelised systems when we discuss Finite automatas and Behavior Trees. The above flow chart is good but as any good robotics programmer knows, we hardly write any code as a sequential structure, the reasons being: 

    - It makes the code fragile to small errors and thus a lot of independent fallbacks and yes, no bifurcations need to be designed.
    - Low modularity makes the code less usable and reusable for the same task in a different algorithm. 
    - Little to no logical bug finding insights. Could you comment from an outcome that was it because of the environment or a fault of the environment? If yes then in how many iterations of testing with hardware(remember this carries entropy with itself{money, time, effort})? And most likely in my opinion no, because of the simple fact that you can't localise weather the logical fault happened in this iteration or the last one. 
  
Thus, we require a better code/framework writing method that helps us with these three deficits and is available to write stuff using opensource frameworks. 


### Finite State Machine
These require no introduction really, a finite set of states that switch based on certain conditions or changes in states. Generally when talking about design of autonomy software architecture, development of individual software autonomy modules and switching between them is a common practice. Essentially a FSM for autonomous system like a robot will be a hybrid automata, i.e. a dynamical system with both discrete and continuous states. We can try to model our decision system defined above as a hybrid automata, here we can have move left, move right and move forward could be the simple action states between which we switch. Now let's judge how our method behaves when judged on our definition of good code: 

    - There could be quick jumps between states thus not making the switches meaningful(in this case since the automata are meant for different purpose).
    - Code modularity is there in the sense that we can write individual behaviors and store them for some other use, however, due to the shortage of robustness to fast switching between logics we see that code needs to take into account fast switching. Which clearly makes it more dependent on the external structure of the high level logic. Which we didn't want. 
    - Bug finding is easy in such small logics, in larger finite automata the number of connections between the nodes explodes, thus making it infeasible to isolate the bug. Also, in a highly connected architecture, it's difficult to say what's failing exactly, note that it's possible to say the node that threw the error is the faulty one, but on a logical level the fault could be coming from some other node that did not fail because environment supported and this caused the system to get into this state with some hidden, unaccounted variables that suddenly start appearing and affecting the low level execution of controls. 

Now, while this simplistic design showed us some faults, is it possible to mitigate with better design, note that currently I am not using any slack variables, is it possible to make the structure stable? Yes, a little but adding more elements to the design adds more complexity generally it is more difficult to change these structures and slack variables just keep adding up as you add more and more nodes. A finite state machine based architecture can be found in FSM's implemented for ROS. Earlier Nav stack used to be state machine based now they have pivoted. 

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/VictorSierra.png" class="Victor-Sierra Search" zoomable=true %}
    </div>
</div>
<div class="caption">
    Fig. 1 Victor-Sierra Search with Datum information.
</div>

#### A more Mathematical Definition and approach to Finite State Machines
A mathematical definition of the finite state machine would be given by a triplet (L,A,E). Where L is the finite set called the state-space, A is the finite set called the alphabet whose elements are called symbols. E is the transition rule; it is a subset of LxAxL and its elements are called edges(or transitions or events)<d-cite key="HDS-Arjan-Hans"></d-cite>. A sequence $$(l_0, a_0, l_1, a_1, ... l_n)$$ is called a trajectory. 

A Deterministic input-output automata can be represented by equations of the following form:
$$
l^{#} = \nu(l,i)
o = \eta(l,i) 
$$

where $$l^{#}$$ denotes the new value of discrete state after the event takes place, resulting from the old discrete state value l and the input i.

A set valued mapping from $$\mathbb{R}^{m}$$, or from a subset S of $$\mathbb{R}^{m}$$, associates, with every point $$ x \in \mathbb{R^m}$$, or every point $$ x \in S $$, a subset of $$ \mathbb{R}^{n}$$. It's notation is double arrow, $$M: \mathbb{R}^{m} \rightrightarrows S$$, for $$S \in \mathbb{R}^{n}$$, indicates that M is a set valued mapping with $$M(x) \in S$$ where S is a subset of $$ \mathbb(R)^m$$.

A hybrid system can thus be defined using 4 elements:
- a set $$ C \in \mathbb{R}^n$$, called the flow set
- a set valued mapping $$ F: \mathbb{R}^n \rightrightarrows \mathbb{R}^n$$ with $$ C \in dom(F)$$, called the flow map;
- a set $$ D \in \mathbb{R}^n$$, called the jump set;
- a set-valued mapping $$ G: \mathbb{R}^n \rightrightarrows \mathbb{R}^n$$ with $$ D \in dom(G)$$, called the jump map;

A hybrid dynamical system can be represented by the notation H = (C,F,D,G). This can be simplified a bit more but I'll post a tutorial on hybrid dynamical systems and that'll have more mathematical content on this topic, with examples. For now this shall do for an intuition on what is going on in a robot and what the model of software running on top of it, should look like.

### Heirarchical Finite State Machines

A state machine of a state machine is called a heirarchical state machine, not really but now you have an intuitive understanding, they are also known as state charts. There are two main definitions required to define a heirarchical finite state machine:

- Super States: group of states that can have transitions. 
- Generalised Transitions: Transitions between Super states

These new design definitions add a bit of nuance to our FSM design example by making us think of things that can be clubbed together. A clear way to do this is by clubbing move left and move right together into move away and we can rewire to show it as shown in the figure:

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/VictorSierra.png" class="Victor-Sierra Search" zoomable=true %}
    </div>
</div>
<div class="caption">
    Fig. 1 Victor-Sierra Search with Datum information.
</div>

This resolves the complexity a little bit, but remember this is a statechart design from a state machine i.e. bottom-up design and not a top to bottom design i.e. we didn't design the behavior and move to the atomic parts. Doing it that way, we can have many ways to model this behavior. 