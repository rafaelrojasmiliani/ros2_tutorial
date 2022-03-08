
- The avaiable reference is [here](https://github.com/ros2/launch/blob/master/launch/doc/source/architecture.rst)
- **Callin convention** In computer science, a calling convention is an implementation-level (low-level) scheme for how subroutines receive parameters from their caller and how they return a result.

- Unix process [live cycle](https://www.cs.miami.edu/home/burt/learning/Csc521.111/notes/process-life-cycle.html) [here](https://www.westfloridacomponents.com/blog/life-cycle-process-linux/) and [here](https://w3.cs.jmu.edu/kirkpams/OpenCSF/Books/csf/html/ProcessCycle.html)

- [example](https://roboticsbackend.com/ros2-launch-file-example/)
- [example](https://github.com/ros2/ros2_documentation/blob/rolling/source/How-To-Guides/Launch-file-different-formats.rst)
- [issue](https://github.com/ros2/launch/issues/117)

- [PathJointSubstituion](https://github.com/ros2/launch/blob/8adf7deccd47eb81d773acf7d2f3c285c1c04bd2/launch/launch/substitutions/path_join_substitution.py#L26) do [this](https://github.com/ros2/launch/blob/8adf7deccd47eb81d773acf7d2f3c285c1c04bd2/launch/launch/substitutions/path_join_substitution.py#L46)

# Relationship Between Nodes and Processes

In ROS 1, there could only ever be one node per process (in ROS 1 nodes and processes are almost interchangeably).
Even for **nodelet**, the conceptual mapping used proxy processes which interacted with roslaunch and OS signals.

The design of ROS2 launch must handle the fact that in ROS2 you can habe many nodes per process.
For example, since there can be multiple nodes per process, shutting down a node no longer always means sending a unix signal to a single process. 

# Parameters

In ROS 1, there was a global parameter which was tightly integrated into roslaunch from ROS 1.
ROS1 also used the other kind of parameters from ROS 1, which were called "dynamic reconfigure parameters".

In ROS 2, there are only one kind of parameters which  work more like “dynamic reconfigure parameters” from ROS 1
in that they are node specific  and they are managed by the node.

# Process Related Events and Responses

Roslaunch from ROS 1 could react only to changes related to a process dead (respawn a process if it died or shutdown the whole launch system if a required process died).

Roslaunch in ROS 2 lets the user specify arbitrary responses to these type of events.

# Deterministic Startup

Roslaunch in ROS1 does not guarantee any particular order to the startup of nodes.

For Managed Nodes, it would not be possible to apply constraints on when something is launched, rather than how it is in roslaunch from ROS 1, where things are run in a non-deterministic order.

The launch system in ROS 2 can model the dependencies between processes and/or nodes where they exist, and the constraints on those dependencies. 
These constraints are not related to ROS events. For example, a user might express that a plain process should be launched after another process has been running for ten seconds. The launch system in ROS 2, could either choose to let the user define a predicate which satisfied that constraint, or it could provide a generic constraint.

# Node Related Events and Responses

The launch system in ROS 2 could export, aggregate and export, or react to lifecycle events of nodes. 

# Static Description and Programmatic API

Most users of roslaunch from ROS 1 used it by defining a static XML description of what they wanted executed and which parameters they wanted to set. 
There is an API for roslaunch in ROS 1, but in our experience few people use this interface.
We can only speculate as to why, but the API is not very well documented and is not prevalent in the tutorials and examples. 
Sticking strictly to the XML description has caused two different approaches to dynamic behavior/configuration to become more popular:

It is a goal of the launch system in ROS 2 to have a more accessible public API to make programmatic approach always an option.

# Locating Files

In the launch system for ROS 2 uses concept of packages to group related resources and programs but it will also support other kinds of relative paths.

# Separation of Concern

The launch system can be considered in parts, separated by concern. The coarse breakdown is like so:

- Calling Conventions for Processes and Various Styles of Nodes
- Reporting System for Events
- System Description and Static Analysis
- Execution and Verification of the System Description
- Testing

## Calling Conventions

Calling conventions describes the interface or contract the launch system has with **anything it is executing and monitoring**.
This contract covers initial execution, activity during runtime, signal handling and behavior of the launch system, and shutdown.

### Operating System Processes

The most basic version of these entities, and the foundation for the other entities, are operating system processes.

For these, the launch system needs to know how to execute them, and to do that it needs:
- name of the executable (just the name, relative path, or absolute path)
- environment variables (`PATH`, `LD_LIBRARY_PATH`...)
- command line arguments
- working directory (directory from which to execute the process)
- launch prefix (used to inject things like gdb, valgrind ...)
- user which should be used to execute the process. 

With this information the launch system can execute any arbitrary operating system process on the local machine.

## Runtime

During runtime, the launch system may monitor all operating system process’s:
- stdout pipe
- stderr pipe
- stdin pipe
- signals (SIGINT, SIGTERM, SIGUSR1, ...)

## Termination

If a process terminates (expected or unexpected) and returns a return code, the launch system can handle the event in a user defined way. 

To terminate a process, the launch system may signal of SIGINT on the child process, after a period of time, signal SIGTERM and after a second period of time, signal SIGKILL.

If the launch system itself receives the SIGTERM signal it will send the SIGKILL signal to all child processes and exit immediately.


# ROS Nodes

Any operating system process can become ROS specific by having at least one ROS Node within it. 
We a process has more than 1 ROS node it has some specific kinds of inputs during execution and it also can affect how the process reacts to signals.

## Execution

Processes with ROS Nodes in them need to consider additional elements, like:
- Package name + executable name rather than executable name + PATH
- ROS specific environment variables (e.g. `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, console output formatting, etc.)
- ROS specific command line arguments
	- Varies for single Node processes and multi Node processes
	- Change node name or namespace
	- Remap topics, services, actions, parameters, etc
	- Initialize parameter values

The launch system is able to take ROS specific declarations and convert them implicitly into normal objecst that a OS-specici process can consume (e.g. environment variables,  command line arguments).

## Runtime

During runtime a ROS node doesn't expose anything new beyond what an operating system process does. 
However ROS nodes implements a signal handler for SIGINT which does a more graceful shutdown.

## Termination

Termination of a ROS Node (**not the process**) is not externally observable beyond what is observed with an operating system process (the return code).

# Managed ROS Nodes

ROS nodes that have a lifecycle have additional runtime states, which the launch system could access and either utilize directly, pass through to the event system, or aggregate before passing it through the event system.

The “Managed ROS Nodes” inherits all of the execution, runtime, and termination characteristics from normal ROS nodes and therefore operating system processes.

## Execution

Managed ROS Nodes do not add any additional inputs or specific configurations at execution time on top of what plain ROS nodes add.

## Runtime

During runtime, a Managed ROS node emits events via topics anytime the state of the node changes. 
These state changes could be consumed by either the launch system itself or by the user, either of which could react to these changes.

## Termination

Managed ROS Nodes have some additional observable effects when terminating (**the node the process containing it**). 
A managed node enters the Finalized state after passing through the ShuttingDown transition state on termination. 
Since these are state transitions, they are observable via the lifecycle event system, at least through the ROS topic `lifecycle_state`.

# Process with a Single (managed or plain) Node

The first is a single process with a single ROS node within it. 
Since there is only one ROS node, the command line arguments do not need to be explicit about to which node they apply. 
For example, changing the namespace of the single node could be expressed with the command line argument `__ns:=new_namespace`.

Even though there is only one node in the process, that node does not need to start when the process starts, nor does the process need to end when the node is shutdown and/or destroyed. If it is a managed node, the lifecycle of the node is best tracked using the lifecycle events. In fact, a process with a single node could start a node, run for a while, later destroy it, and then create it again.

**So the biggest impact of a single node process is that the configuration, in terms of command line arguments and environment variables, can be simplified.**

# Process with Multiple Nodes

In a process with multiple nodes, things are much the same as with a process with a single node, but the configuration, again in terms of command line arguments and environment variables, need to be more specific in order to discriminate between the various nodes being instantiated in the process. 

However, as an example of a process with multiple nodes, consider a program that instantiates two camera driver nodes called `camera1` and `camera2` by default. 
You could configure their namespaces separately by doing something like `camera1:__ns:=left camera2:__ns:=right`.

- **Dynamically loading Nodes** Dynamically loading a node means spawning it in a process that does not know about the node until it is asked to load it. 

### Container Process API
While there will be standard container processes, custom container processes would allow using custom executors or client libraries. Therefore, there must be a container process API for the launch system to communicate which nodes should be loaded.
The launch system must be able tell the container process what arguments to give to a dynamically loaded node. This includes command line arguments and client library specific options (e.g. rclcpp has `use_intra_process_comms`). Since the launch system cannot know about all custom containers, the API must include a way to pass unknown arguments (e.g. by passing key-value pairs).


# Launch System Events

The most basic events are related solely to things that happen within the launch system itself. This can be as simple as a timed event, either a specific amount of time has passed, or a specific time of day has passed, or an “idle” event which might be used to implement a “call later” type of callback.

However, these events can be more specific to the launch system, like when a launch description is included, or when the launch system needs to shutdown. These events might also contain pertinent information like why a launch description was included, e.g. given as an argument to the launch system, included by another launch file, requested to be included by asynchronous request (maybe via a ROS service call), or in the case of a shutting down event, maybe why the launch system is shutting down, e.g. a required process exited, or it received the SIGINT signal.
Operating System Events

Other events will be specific to any process that is executed by the launch system, like when a process is started or when a process exits. You could also imagine events which get fired when stdout or stderr data is received from the process by the launch system, assuming it captures that information.
ROS Specific Events

ROS specific events would most likely occur in processes that launch is executing, but using ROS topics and/or services launch could observe these events and generate equivalent events within the launch event system. For example, if a process being run by launch contains a node with a life cycle, launch could observe any life cycle state transitions the node makes and create an event each time one of those transitions occur. Using this a user could, for example, wait for a node to reach the “active” state and only then start another process.
Reporting and Handling of Events

Without getting into implementation details (e.g. libraries and class hierarchies), this subsection will try to express what information should be contained within events, and how they can be accessed and what the behavior of the event system is with respect to delivery.
Information Provided by Events

Like many other event systems, the events should be capable of not only notifying that an event has occurred, but it should be able to communicate data associated with the event.

A simple example of an event without extra data might be an event for “call later”, where it doesn’t matter who initiated the “call later” or how long it has been since that occurred (though it could include that if it wished), and so this events existence is sufficient to notify waiting actions to proceed.

A simple example of an event with extra data might be a “process exited” event, which would include enough information to identify which process it was as well as the return code of the process.

Again, like many other event systems, the events should have a type (either as an attribute or as a child class) which can be used to filter events to particular handlers.
Event Handlers

Events can be handled by registering an event handler with the launch system. The only required form of event handler is one that is a function, registered locally with the launch system.

Other kinds of event handlers could be supported by building on a locally defined function. They could be something like a user-defined “lambda” defined in the description of the launch file, or even a built-in event handler function which just publishes the events as ROS messages. In the latter case, it could be either be a subscription to a topic (which needs no a priori registration with the launch system) or a service call (which was registered with the launch system a priori). So if it is a topic, the subscription object, with its callback, could be considered an event handler. In the case of a service, which would be called by the launch system and handled by a user defined service server, the service server (and it’s response) would be considered the event handler.
Handling of Events

By default, events are passed to all event handlers and there is no way for an event handler to “accept” an event to prevent it from being delivered to other events.

While event handlers have no comparison operators between one another (so no sorting), the order of delivery of events to event handlers should be deterministic and should be in the reverse order of registration, i.e. “first registered, last delivered”. Note that delivery to asynchronous event handlers (e.g. a subscription to a ROS topic for events, sent via a ROS publisher), will be sent in order, but not necessarily delivered in order.
Event Filters

Like the Qt event system, it will be possible to create event filters, which emulate the ability to accept events and prevent them from being sent “downstream”. 6

Unlike the Qt event system, an event filter is simply like any other event handler, and will not prevent other event handlers from receiving the event. Instead, each event filter will have its own list of event handlers, each of which can accept or reject an event, allowing or denying further processing of the event within the event filter, respectively.

Any event handler can be added to an event filter, but “pure event sinks” are unable to accept an event, e.g. things like a ROS topic. This is because there is no feedback mechanism, i.e. a subscription to a topic has no way of indicating if an event has been accepted or rejected as it does not have a return type. Whereas, other event handlers which are functions or lambda’s withing the launch system itself or ROS service calls can have a return type and therefore can accept or reject an event.
Sending Events

It should be possible for users of the launch system send events, in addition to the system being able to do so itself.
System Description

The system description is a declarative set of actions and reactions that describe what the user wants to launch in a format that the launch system can interpret.

The goal of the system description is to capture the intentions of the user describing the system to be launched, with as few side effects as possible. The reason for doing this is so that a launch description can be visualized and statically analyzed without actually launching the described system. Having a tool that can allow a developer to visualize and modify the launch description in a WYSIWYG (what you see is what you get) editor is an important use case for the system description.

First, this section will describe in a programming language, or text markup, agnostic way what can be expressed in the system description and how it maps to the calling conventions and event handling described in previous sections, as well as how it maps to launch system specific behaviors. After that, it will suggest how this agnostic system description can be applied to Python and XML, but also how it might be able to be extended to support other languages and markups.
Launch Descriptions

The system is described in parts which we’ll refer to here as “Launch Descriptions”. Launch descriptions are made of up of an ordered list of actions and groups of actions. It may also contain substitutions throughout the description, which are used to add some flexibility and configuration to the descriptions in a structured way.
Actions

Actions may be one of several things, and each action has a type (associated with the action’s function) and may also contain arbitrary configurations. Actions represent an intention to do something, but a launch description is parsed first, then actions are taken in order of definition later. This allows actions to be interpreted and then statically introspected without actually executing any of them unless desired.

By separating the declaration of an action from the execution of an action, tools may use the launch descriptions to do things like visualize what a launch description will do without actually doing it. The launch system will simply use the interpreted actions in the launch descriptions to actually execute the actions.

Basic actions include being able to:

    include another launch description
    modify the launch system configurations at the current scope
    execute a process
    register/unregister an event handler
    emit an event
    additional actions defined by extensions to the launch system

Actions may also yield more actions and groups rather than perform an actual task. For example, an action to “run a node” may end up resulting in “executing two process” or in “executing a process and registering an event handler”. These kind of actions could be thought of a launch description generators or macros, since they effectively generate the same contents as a launch description, but look like an action to the user.

This allows for more complex actions which might include, but not be limited to:

    include a launch description from a file with a certain markup type
    set an environment variable
    run a single-node process
    run a multi-node process
    run a node container
    run a node proxy to load into a node container
    run a process on a remote computer
    declare launch description arguments
        exposed as either:
            command line arguments for top-level launch descriptions
            or additional arguments to the “include another launch description” action
        stored in “launch system configuration”
    various OS actions, e.g. touch a file, read a file, write to a file, etc…

Each of these actions would be able to generate one or more other actions. This can be used to run one or more processes with a single action statement, or to simply provide some “syntactic sugar” For example, a “run a single-node process” action might take ROS specific configurations, then expand them to generic configurations for one of the basic actions like the “execute a process” action.
Including Another Launch Description

One of the simplest actions is to include another launch description. This launch description is processed in its entirety, including parsing of any launch descriptions it includes recursively. Therefore processing of launch descriptions is in order, and depth first.

Included launch descriptions inherit all configurations of the current launch description, and any changes to the launch system configurations made in the included launch description will affect actions after the include action.

However, it should also be possible to control which configurations are inherited by an included launch description and also to “scope” an included launch description so that it cannot affect the configuration above it.

In all cases, the desired behavior may be achieved though selective use of optionally scoped group actions.
Launch System Configuration

The “modify the launch system configurations at the current scope” action mentioned above is able to mutate a local scope of configurations which can affect other actions which come after the modification. Actions may use this local state to uniformly apply certain settings to themselves.

For example, the environment variables which are set when running an operating system process would be taken from the launch system configuration, and therefore can be modified with an action. Then any “execute a process” actions which come after it will be affected by the configuration change.

Changes to the local state by included launch descriptions persist, as they should be thought of as truly included in the same file, as if you had copied the contents of the included launch description in place of the include action. To avoid this, a group without a namespace could be used to produce a push-pop effect on the launch configurations.
Execute a Process

Another basic action would be to execute a subprocess, with arguments and emitted events, as described in the calling conventions section under “operating system process”.

This action will take a few required arguments, a few optional requirements, and also take settings from the launch system configurations if they’re not explicitly given. The signature of this action should be similar to the API of Python’s subprocess.run function7. Basically taking things like the executable file, arguments, working directory, environment, etc. as input and reporting the return code, stdout and stderr, and any errors as emitted events.

Also, every executed process will automatically setup a few event handlers, so that the user can emit events to ask the launch system to terminate the process (following the signal escalation described in previous sections), signal the process explicitly, or write to the stdin of the process. More sophisticated calling conventions which are based on the “operating system process” may include other default event handlers.
Event Handlers

The launch description can also contain event handlers. An event handler is essentially a function which takes an event as input and returns a launch description to be included at the location of the event handler registration. The event handler will be executed asynchronously when the associated event is emitted.

There are two actions associated with event handlers, registering one and unregistering one. How event types and event handlers are represented and tracked depends on the implementation of the launch system. However, as an example, a “launch file” written in Python might represent event’s as classes which inherit from a base class. If instead the “launch file” is written in XML, event types might be expressed as a string, with launch system events using a “well-known name” and with user definable events being represented with unique strings as well. Similarly, the Python based “launch file” might use instances of objects to represent registered event handlers, therefore you might need that object to perform the unregister action. And in the XML based “launch file” the user might be required to give a unique name to all registered event handlers, so that they can unregistered with the same name later.

When an event handler finishes, it is able to return a launch description which is implicitly given to the include action at the location of the event handler’s registration. This allows an event handler to cause any action upon completion, e.g. include another launch description, unregister an event handler, emit another event, run a process, start the termination of a process by emitting an event, etc…

The lowest level of event handlers is the function which takes an event and returns a launch description. For example, a user defined event handler might look like this in Python:

# This is a made up example of an API, consider it pseudo code...

launch_description = LaunchDescription(...)
# ...

def my_process_exit_logger_callback(event: ProcessExitedEvent) -> LaunchDescription:
    print(f"process with pid '{event.pid}' exited with return code '{event.return_code}'")

launch_description.register_event_handler(
    ProcessExitedEvent, my_process_exit_logger_callback, name='my_process_exit_logger')

However, to remove boilerplate code or to avoid programming in markup descriptions, common event handler patterns can be encapsulated in different event handler signatures.

For example, there might be the on_event event handler signature, which then returns a given set of actions or groups the user provides. This signature might be useful to after ten seconds start a node or include another launch file, and in XML it might look like this:

<!-- This is a made up example of a markup, consider it pseudo code... -->
<!-- Also this could be made even simpler by just having a tag which lets -->
<!-- you specify the extra actions directly, rather than emitting an event and -->
<!-- handling it, but this demonstrates the custom event handler signature -->

<emit_event after="10" type="my_custom_timer_event" />

<on_event event_type="my_custom_timer_event">
  <!-- actions to be included when this event occurs -->
  <include file="$(package-share my_package)/something.launch.xml" />
  <node pkg="my_package" executable="my_exec" />
  <group namespace="my_ns">
    <node pkg="my_package" executable="my_exec" />
  </group>
</on_event>

Emitting Events

Another basic action that the launch system should support is the ability to emit events and if necessary declare new kinds of events before emitting them.

This feature could be used by users to filter a launch system event and then dispatch it to other user defined event handlers, or to create new or modified events based on existing events.

How events are defined is up to the implementation, but it should be possible to model the events so they can be emitted and then handled by registered event handlers.
Groups

TODO:

    can be broken into:
        “namespace” (like roslaunch),
        conditionals (if and unless) (see: https://wiki.ros.org/roslaunch/XML#if_and_unless_attributes), and
        scope (push-pop for configurations)
        should consider what we’re discussing to do in https://github.com/ros2/launch/issues/313

Substitutions

TODO:

    equivalent to substitutions in ROS 1, see: https://wiki.ros.org/roslaunch/XML#substitution_args
    they’ve already been implemented in the reference implementation, they should at least be summarized as built here

Mapping to Programming Languages and Markup Languages

TODO:

    Explain in general how the features described in the previous sections would map to a programming language and/or markup language and any considerations therein.
    How it would map to Python (likely implementation)
    How it would map to XML (likely first markup language)

Execution and Verification of the System Description

TODO: Restructure notes on this and put them here.

Temporary summary:

Whether described via static file or programmatically, once the system is described it has to be executed, and this section will cover all of that. Most of this is already covered in the “calling conventions” section, but this section will also cover some more details about execution, and then add on to that verification (starting another discussion about what the launch system should and should not do itself). Verification is runtime assertion that mirrors the static analysis that can be done off-line.
Testing

TODO: Restructure notes on this and put them here.

Temporary summary:

In ROS 1, rostest is an important extension of roslaunch, and so far in ROS 2 we’re already using the foundation of launching (executing processes and reacting to their exit, return codes, and stdout/stderr), called ros2/launch_testing right now, to implement some tests. This section will cover how that happens and how it integrates with the static description files as well as the programmatic API, adding ROS specific concepts to what we’re already doing with ros2/launch_testing.
Requirements

TODO: Reformat requirements list, possibly combine/reconcile with “separation of concerns section” (consider dropping in favor of renaming to something that implies requirements as well)
Reference Implementation Proposal

TODO: This will outline what we have and what we need to build and how it should be separated.
Alternatives

TODO: Anything we choose not to support in the requirements vs. the “separation of concern section”, and also any alternatives which we considered but rejected in the reference implementation proposal.
References

    
The git repo is [here](https://github.com/ros2/launch).

Tutorial [here](https://docs.ros.org/en/galactic/Tutorials/Launch/Creating-Launch-Files.html)

Motivation [here](https://design.ros2.org/articles/roslaunch.html)

- [ros2](https://github.com/ros2/ros2cli/blob/f93e41a6ce6b968d28e5f859e12eca24cca42665/ros2cli/setup.py#L61) definiton of ros2 command line script at insalltion level
- [cli](https://github.com/ros2/ros2cli/blob/master/ros2cli/ros2cli/cli.py)
- [ros2launch](https://github.com/ros2/launch_ros/tree/galactic/ros2launch) [here](https://github.com/ros2/launch_ros.git)

- [ros2 launch](https://github.com/ros2/launch_ros/blob/galactic/ros2launch/ros2launch/command/launch.py)

- [`launch_a_launch_file`](https://github.com/ros2/launch_ros/blob/7dfb3a09c1c80979be5678394bd00aec43e25279/ros2launch/ros2launch/api/api.py#L143)

- [docu](https://roboticsbackend.com/ros2-launch-file-example/)
- [docu](https://ubuntu.com/blog/ros2-launch-required-nodes)
