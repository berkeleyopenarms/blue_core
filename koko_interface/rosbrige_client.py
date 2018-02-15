"""This module provides a python client for rosbridge to publish, subscribe topics,
call services and use action client.
"""
 
import threading
import time
import json
import uuid
from ws4py.client.threadedclient import WebSocketClient
from pydispatch import dispatcher
 
 
class ROSBridgeClient(WebSocketClient):
    """ROSBridgeClient extends WebSocketClient and manages connection to the server and all interactions with ROS.
 
    It keeps a record of all publishers, subscriber, service request callbacks and action clients.
    """
    def __init__(self, ip, port=9090):
        """Constructor for ROSBridgeClient
 
        Args:
            ip (str): The robot IP address.
            port (int, optional): The WebSocket port number for rosbridge. Defaults to 9090.
        """
        WebSocketClient.__init__(self, 'ws://{}:{}'.format(ip, port))
        self._connected = False
        self._id_counter = 0
        self._publishers = {}
        self._subscribers = {}
        self._services = {}
        self._action_clients = {}
        self.connect()
        threading.Thread(target=self.run_forever).start()
        while not self._connected:
            time.sleep(0.1)
 
    @property
    def id_counter(self):
        """Generate an auto-incremental ID starts from 1.
 
        Returns:
            A auto-incremented ID.
        """
        self._id_counter += 1
        return self._id_counter
 
    def publisher(self, topic_name, message_type, latch=False, queue_size=1):
        """Create a _Publisher object if the given topic hasn't been advertised, otherwise return the existing
        publisher that is currently advertising the topic.
 
        Args:
            topic_name (str): The ROS topic name.
            message_type (str): The ROS message type, such as `std_msgs/String`.
            latch (bool, optional): Whether the topic is latched when publishing. Defaults to False.
            queue_size (int): The queue created at bridge side for re-publishing. Defaults to 1.
 
        Returns:
            A _Publisher object.
        """
        if topic_name in self._publishers:
            publisher = self._publishers.get(topic_name)
            publisher.usage += 1
        else:
            print('Advertising topic {} for publishing'.format(topic_name))
            publisher = _Publisher(self, topic_name, message_type, latch, queue_size)
            self._publishers[topic_name] = publisher
        return publisher
 
    def unregister_publisher(self, topic_name):
        """Stop advertising on the given topic.
 
        Args:
            topic_name (str): The ROS topic name.
        """
        if topic_name in self._publishers:
            print('Stop advertising topic {} for publishing'.format(topic_name))
            del self._publishers[topic_name]
 
    def subscriber(self, topic_name, message_type, cb):
        """Create a _Subscriber object on a given topic with a callback function.
 
        If the topic hasn't been subscribed yet, subscribe the topic. Otherwise, it adds the subscriber
        with callback function into the topic subscription list.
 
        Args:
            topic_name (str): The ROS topic name.
            message_type (str): The ROS message type, such as `std_msgs/String`.
            cb (function): A function will be called when a message is received on that topic.
 
        Returns:
            A _Subscriber object.
        """
        subscriber = _Subscriber(self, topic_name, cb)
        if topic_name in self._subscribers:
            self._subscribers.get(topic_name).get('subscribers').append(subscriber)
        else:
            subscribe_id = 'subscribe:{}:{}'.format(topic_name, self._id_counter)
            print('Sending request to subscribe topic {}'.format(topic_name))
            self.send(json.dumps({
                'op': 'subscribe',
                'id': subscribe_id,
                'topic': topic_name,
                'type': message_type
            }))
            self._subscribers[topic_name] = {}
            self._subscribers[topic_name]['subscribe_id'] = subscribe_id
            self._subscribers[topic_name]['subscribers'] = [subscriber]
        return subscriber
 
    def unsubscribe(self, subscriber):
        """Remove a callback subscriber from its topic subscription list.
 
        If there is no callback subscribers in the subscription list. It will unsubscribe the topic.
 
        Args:
            subscriber (_Subscriber): A subscriber with callback function that listen to the topic.
        """
        topic_name = subscriber.topic_name
        if topic_name not in self._subscribers:
            return
        subscribe_id = self._subscribers.get(topic_name).get('subscribe_id')
        subscribers = self._subscribers.get(topic_name).get('subscribers')
        if subscriber in subscribers:
            subscribers.remove(subscriber)
        if len(subscribers) == 0:
            print('Sending request to unsubscribe topic {}'.format(topic_name))
            del subscribers[:]
            self.send(json.dumps({
                'op': 'unsubscribe',
                'id': subscribe_id,
                'topic': topic_name
            }))
            del self._subscribers[topic_name]
 
    def service(self, service_name, service_type):
        """Create a ROS service client.
 
        Args:
            service_name (str): The ROS service name.
            service_type (str): The ROS service type.
 
        Returns:
            A _Service object.
        """
        return _Service(self, service_name, service_type)
 
    def register_service_callback(self, service_id, cb):
        """Register a service callback with a service request ID.
 
        Args:
            service_id (str): The service request ID.
            cb (function): A function will be called when the service server responses.
        """
        self._services[service_id] = cb
 
    def action_client(self, server_name, action_name):
        """Create a ROS action client if there was no client created for the action server.
        Otherwise return that action client.
 
        Args:
            server_name (str): The ROS action server name.
            action_name (str): The ROS action name.
 
        Returns:
            A _ActionClient object.
        """
        if server_name + ':' + action_name in self._action_clients:
            action_client = self._action_clients.get(server_name + ':' + action_name).get('action_client')
            action_client.usage += 1
        else:
            action_client = _ActionClient(self, server_name, action_name)
        return action_client
 
    def unregister_action_client(self, server_name, action_name):
        """Unregister the action client with server and action name. Remove it from the internal action client dict.
 
        Args:
            server_name (str): The ROS action server name.
            action_name (str): The ROS action name.
        """
        if server_name + ':' + action_name in self._action_clients:
            del self._action_clients[server_name + ':' + action_name]
 
    def opened(self):
        """Called when the connection to ROS established."""
        self._connected = True
        print('Connected with rosbridge')
 
    def closed(self, code, reason=None):
        """Called when the connection to ROS disconnected
 
        Args:
            code (int): A status code.
            reason (str, opitonal): A human readable message. Defaults to None.
        """
        print('Disconnected with rosbridge')
 
    def received_message(self, message):
        """Called when message received from ROS server.
 
        Only handle the message with `topic` or `service` keywords and trigger corresponding callback functions.
 
        Args:
            message(ws4py.messaging.Message): A message that sent from ROS server.
        """
        data = json.loads(message.data)
        if 'topic' in data:
            dispatcher.send(data.get('topic'), message=data.get('msg'))
        if 'service' in data:
            service_id = data.get('id')
            success = data.get('result')
            values = data.get('values')
            if service_id in self._services:
                self._services[service_id](success, values)
                del self._services[service_id]
 
    def unhandled_error(self, error):
        """Called when a socket or OS error is raised.
 
        Args:
            error (str): A human readable error message.
        """
        print(error)
 
 
class _Publisher(object):
    def __init__(self, rosbridge, topic_name, message_type, latch=False, queue_size=1):
        """Constructor for _Publisher.
 
        Args:
            rosbridge (ROSBridgeClient): The ROSBridgeClient object.
            topic_name (str): The ROS topic name.
            message_type (str): The ROS message type, such as `std_msgs/String`.
            latch (bool, optional): Whether the topic is latched when publishing. Defaults to False.
            queue_size (int): The queue created at bridge side for re-publishing. Defaults to 1.
        """
        self._advertise_id = 'advertise:{}:{}'.format(topic_name, rosbridge.id_counter)
        self._rosbridge = rosbridge
        self._topic_name = topic_name
        self._usage = 1
 
        rosbridge.send(json.dumps({
            'op': 'advertise',
            'id': self._advertise_id,
            'topic': topic_name,
            'type': message_type,
            'latch': latch,
            'queue_size': queue_size
        }))
 
    @property
    def usage(self):
        return self._usage
 
    @usage.setter
    def usage(self, value):
        self._usage = value
 
    def publish(self, message):
        """Publish a ROS message
 
        Args:
            message (dict): A message to send.
        """
        self._rosbridge.send(json.dumps({
            'op': 'publish',
            'id': 'publish:{}:{}'.format(self._topic_name, self._rosbridge.id_counter),
            'topic': self._topic_name,
            'msg': message
        }))
 
    def unregister(self):
        """Reduce the usage of the publisher. If the usage is 0, unadvertise this topic."""
        self._usage -= 1
        if self._usage == 0:
            self._rosbridge.unregister_publisher(self._topic_name)
            self._rosbridge.send(json.dumps({
                'op': 'unadvertise',
                'id': self._advertise_id,
                'topic': self._topic_name
            }))
 
 
class _Subscriber(object):
    def __init__(self, rosbridge, topic_name, cb=None):
        """Constructor for _Subscriber.
 
        Args:
            rosbridge (ROSBridgeClient): The ROSBridgeClient object.
            topic_name (str): The ROS topic name.
            cb (function): A function will be called when a message is received on that topic.
        """
        self._rosbridge = rosbridge
        self._topic_name = topic_name
        self._cb = cb
        if callable(self._cb):
            dispatcher.connect(self._cb, signal=topic_name)
 
    @property
    def topic_name(self):
        return self._topic_name
 
    def unregister(self):
        """Remove the current callback function from listening to the topic,
        and from the rosbridge client subscription list
        """
        if callable(self._cb):
            dispatcher.disconnect(self._cb, signal=self._topic_name)
        self._rosbridge.unsubscribe(self)
 
 
class _Service(object):
    def __init__(self, rosbridge, service_name, service_type):
        """Constructor for _Service.
 
        Args:
            rosbridge (ROSBridgeClient): The ROSBridgeClient object.
            service_name (str): The ROS service name.
            service_type (str): The ROS service type.
        """
        self._rosbridge = rosbridge
        self._service_name = service_name
        self._service_type = service_type
 
    def request(self, request, cb):
        """Send a request to the ROS service server. The callback function will be called when service responses.
 
        Args:
            request (dict): A request message to send,
            cb (function): A function will be called when the service server responses.
 
        Returns:
 
        """
        service_id = 'call_service:{}:{}'.format(self._service_name, self._rosbridge.id_counter)
        if callable(cb):
            self._rosbridge.register_service_callback(service_id, cb)
        self._rosbridge.send(json.dumps({
            'op': 'call_service',
            'id': service_id,
            'service': self._service_name,
            'args': request
        }))
 
 
class _ActionClient(object):
    def __init__(self, rosbridge, server_name, action_name):
        """Constructor for _ActionClient
 
        Args:
            rosbridge (ROSBridgeClient): The ROSBridgeClient object.
            server_name (str): The ROS action server name.
            action_name (str): The ROS action name.
        """
        self._rosbridge = rosbridge
        self._server_name = server_name
        self._action_name = action_name
        self._goals = {}
        self._usage = 1
 
        self._feedback_sub = rosbridge.subscriber(server_name+'/feedback', action_name+'Feedback', self.on_feedback)
        self._result_sub = rosbridge.subscriber(server_name+'/result', action_name+'Result', self.on_result)
 
        self._goal_pub = rosbridge.publisher(server_name+'/goal', action_name+'Goal')
        self._cancel_pub = rosbridge.publisher(server_name+'/cancel', 'actionlib_msgs/GoalID')
 
    @property
    def usage(self):
        return self._usage
 
    @usage.setter
    def usage(self, value):
        self._usage = value
 
    def on_feedback(self, message):
        """Callback when a feedback message received.
 
        Args:
            message (dict): A feedback message received from ROS action server.
        """
        goal = self._goals.get(message.get('status').get('goal_id').get('id'))
        if goal:
            goal.feedback_received(message.get('feedback'), message.get('status'))
 
    def on_result(self, message):
        """Callback when a result message received.
 
        Args:
            message (dict): A result message received from ROS action server.
        """
        goal = self._goals.get(message.get('status').get('goal_id').get('id'))
        if goal:
            goal.result_received(message.get('result'), message.get('status'))
 
    def send_goal(self, goal_message, on_result, on_feedback):
        """Send a goal to the ROS action server.
 
        Args:
            goal_message (dict): A message to send to ROS action server.
            on_result (function): A callback function to be called when a feedback message received.
            on_feedback (function): A callback function to be called when a result message received.
        """
        goal = _Goal(goal_message, on_result, on_feedback)
        self._goals[goal.id] = goal
        self._goal_pub.publish(goal.message)
        return goal.id
 
    def cancel_goal(self, goal_id):
        """Cancel a goal with a given goal ID
 
        Args:
            goal_id (str): The ID of the goal to be cancelled.
        """
        self._cancel_pub.publish({'id': goal_id})
 
    def unregister(self):
        """Reduce the usage of the action client. If the usage is 0, unregister its publishers and subscribers."""
        self._usage -= 1
        if self._usage == 0:
            self._feedback_sub.unregister()
            self._result_sub.unregister()
            self._goal_pub.unregister()
            self._cancel_pub.unregister()
            self._rosbridge.unregister_action_client(self._server_name, self._action_name)
 
 
class _Goal(object):
    def __init__(self, message, on_result=None, on_feedback=None):
        """Constructor for _Goal
 
        Args:
            message (dict): The goal message to send to ROS action server.
            on_result (function): A callback function to be called when a feedback message received.
            on_feedback (function): A callback function to be called when a result message received.
        """
        self._id = 'goal_' + str(uuid.uuid4())
        self._message = message
        self._is_finished = False
        self._on_result = on_result
        self._on_feedback = on_feedback
 
    @property
    def id(self):
        return self._id
 
    @property
    def message(self):
        """Wrap message in JSON format that complies ROSBridge protocol.
 
        Returns:
            A Json that contains the goal ID and message.
        """
        return {
            'goal_id': {
                'stamp': {
                    'secs': 0,
                    'nsecs': 0
                },
                'id': self._id
            },
            'goal': self._message
        }
 
    @property
    def is_finished(self):
        return self._is_finished
 
    def result_received(self, result, status):
        """Called when a result message is received.
 
        Args:
            result (dict): The result message.
            status (int): The status code. Such as:
                ACTIVE = 1: The goal is currently being processed by the action server;
                PREEMPTED = 2: The goal received a cancel request after it started executing;
                SUCCEEDED = 3: The goal was achieved successfully by the action server;
                ABORTED = 4: The goal was aborted during execution by the action server due to some failure.
                For more details, refer to http://docs.ros.org/indigo/api/actionlib_msgs/html/msg/GoalStatus.html.
        """
        self._is_finished = True
        if callable(self._on_result):
            self._on_result(result, status)
 
    def feedback_received(self, feedback, status):
        """Called when a result message is received.
 
        Args:
            feedback (dict): The feedback message.
            status (int): The status code. Such as:
                ACTIVE = 1: The goal is currently being processed by the action server;
                PREEMPTED = 2: The goal received a cancel request after it started executing;
                SUCCEEDED = 3: The goal was achieved successfully by the action server;
                ABORTED = 4: The goal was aborted during execution by the action server due to some failure.
                For more details, refer to http://docs.ros.org/indigo/api/actionlib_msgs/html/msg/GoalStatus.html.
        """
        if callable(self._on_feedback):
            self._on_feedback(feedback, status)
