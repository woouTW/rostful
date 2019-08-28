# -*- coding: utf-8 -*-
from __future__ import absolute_import

import re
import sys

# Reference for package structure since this is a flask app : http://flask.pocoo.org/docs/0.10/patterns/packages/
from rostful import get_pyros_client
import dynamic_reconfigure.client
import rospy
import yaml

import time

CONFIG_PATH = '_rosdef'
SRV_PATH = '_srv'
MSG_PATH = '_msg'


def get_suffix(path):
    suffixes = '|'.join([re.escape(s) for s in [CONFIG_PATH, SRV_PATH, MSG_PATH]])
    match = re.search(r'/(%s)$' % suffixes, path)
    return match.group(1) if match else ''

# TODO : remove ROS usage here, keep this a pure Flask App as much as possible

import simplejson
import tblib

try:
    # Python 2
    from cStringIO import StringIO
except ImportError:
    # Python 3
    from io import StringIO


ROS_MSG_MIMETYPE = 'application/vnd.ros.msg'
def ROS_MSG_MIMETYPE_WITH_TYPE(rostype):
    if isinstance(rostype, type):
        name = rostype.__name__
        module = rostype.__module__.split('.')[0]
        rostype = module + '/' + name
    return 'application/vnd.ros.msg; type=%s' % rostype


#req should be a flask request
#TODO : improve package design...
def request_wants_ros(req):
    best = req.accept_mimetypes.best_match([ROS_MSG_MIMETYPE,'application/json'])
    return best == ROS_MSG_MIMETYPE and req.accept_mimetypes[best] > req.accept_mimetypes['application/json']
#implementation ref : http://flask.pocoo.org/snippets/45/


def get_json_bool(b):
    if b:
        return 'true'
    else:
        return 'false'


def get_query_bool(query_string, param_name):
    return re.search(r'(^|&)%s((=(true|1))|&|$)' % param_name, query_string, re.IGNORECASE)


def make_dict(**kwargs):
    result = {}
    for key, val in kwargs.items():
        result[key] = val
    return result


def find_rule(node_name, descriptions):
    for description in descriptions:
        if description["name"] == node_name:
            return description
    return None


def string_type_check(string):
    try:
        float(string)
        if float(string) == int(float(string)):
            return int
        else:
            return float
    except:
        return str

    


from flask import request, make_response, render_template, jsonify, redirect
from flask.views import MethodView
from flask_restful import reqparse
import flask_restful as restful

from . import api, api_blueprint, current_app

from webargs.flaskparser import FlaskParser, use_kwargs

parser = FlaskParser()

from pyros_common.exceptions import PyrosException

from rostful.exceptions import ServiceNotFound, ServiceTimeout, WrongMessageFormat

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

import os


class Timeout(object):
    """
    Small useful timeout class
    """
    def __init__(self, seconds):
        self.seconds = seconds

    def __enter__(self):
        self.die_after = time.time() + self.seconds
        return self

    def __exit__(self, type, value, traceback):
        pass

    @property
    def timed_out(self):
        return time.time() > self.die_after


# TODO : check if we can simplify this by dynamically generating usual simple flask route (one for each service/topic)
# this way we could use url_for simply and rely on flask url build scheme...
#@api.resource('/', '/<path:rosname>', strict_slashes=False)
# TO FIX flask restful problem
# TODO : get rid of flask restful dependency, it is apparently not maintained any longer
# @api.route('/')
# @api.route('/<path:rosname>')
class BackEnd(restful.Resource):   # TODO : unit test that stuff !!! http://flask.pocoo.org/docs/0.10/testing/
    """
    View for backend pages
    """
    def __init__(self, rosname=None):
        self.start_time = time.time()
        def null_function(config):
            pass
        super(BackEnd, self).__init__()

        self.node_client = get_pyros_client()  # we retrieve pyros client from app context

        self.is_dr = {}

        # dynamic import
        from pyros_interfaces_ros import definitions
        from pyros.client.client import PyrosServiceTimeout, PyrosServiceNotFound

    # TODO: think about login rest service before disabling REST services if not logged in
    def get(self, rosname=None):
        def type_adopt(type_str):
            if type_str == "str":
                return "string"
            elif type_str == "bool":
                return "bool"
            elif type_str in ["int", "float"]:
                return type_str+"32"
            elif type_str == "list":
                return "string[]"
            
        def param_adopt(param):
            param = dict(param)
            param_name = rosname.split("/")[1]
            type_name = type_adopt(type(param["prmtype"]).__name__)
            param["msgtype"] = {param_name:type_name}
            return param

        current_app.logger.debug('in BackEnd with rosname: %r', rosname)

        # dynamic import
        from pyros_interfaces_ros import definitions
        from pyros.client.client import PyrosServiceTimeout, PyrosServiceNotFound

        # TODO : replace this with webargs ( less buggy )
        parser = reqparse.RequestParser()
        # somehow this breaks requests now... disabling for now.
        # parser.add_argument('full', type=bool)
        # parser.add_argument('json', type=bool)
        args = parser.parse_args()

        path = '/' + (rosname or '')
        full = args.get('full', True)
        jsn = args.get('json', True)

        suffix = get_suffix(path)

        # fail early if no pyros client
        if self.node_client is None:
            current_app.logger.warn('404 : %s', path)
            return make_response('', 404)

        services = self.node_client.services()
        topics = self.node_client.topics()
        params = self.node_client.params()
        for key in list(params.keys()):
            val = params[key]
            if type(val).__name__ != "dict":
                self.is_dr[key] = True
                params[key] = param_adopt(dict(val))
            else:
                self.is_dr[key] = False

        # Handling special case empty rosname and suffix
        if path == '/' and not suffix:
            return jsonify({
                "services": services,
                "topics": topics,
                "params": params
            })

        # special case to get rosdef of all services and topics
        if path == CONFIG_PATH:
            cfg_resp = None
            dfile = definitions.manifest(services, topics, full=full)
            if jsn:
                cfg_resp = make_response(str(dfile.tojson()), 200)
                cfg_resp.mimetype = 'application/json'
            else:
                cfg_resp = make_response(dfile.tostring(suppress_formats=True), 200)
                cfg_resp.mimetype='text/plain'
            return cfg_resp

        if not suffix:
            if params is not None and path in params:
                msg = self.node_client.param_get(path)
            elif services is not None and path in services:
                msg = self.node_client.service_call(path)
            elif topics is not None and path in topics:
                msg = self.node_client.topic_extract(path)
            else:
                current_app.logger.warn('404 : %s', path)
                msg = make_dict(description="not found name", result={"error":1})
                output_data = simplejson.dumps(msg, ignore_nan=True)
                return make_response(output_data, 404)

            #current_app.logger.debug('mimetypes : %s', request.accept_mimetypes)

            if msg is None:
                return make_response('', 204)  # returning no content if the message is not there
                # different than empty {} message

            # This changes nan to null (instead of default json encoder that changes nan to invalid json NaN).
            # some client might be picky and this should probably be a configuration setting...
            msg = make_dict(description="succeed", result={"value":msg})
            output_data = simplejson.dumps(msg, ignore_nan=True)
            mime_type = 'application/json'

            #if request_wants_ros(request):
            #    mime_type = ROS_MSG_MIMETYPE
            #    output_data = StringIO()
            #    if msg is not None:
            #        msg.serialize(output_data)
            #    output_data = output_data.getvalue()
            #else:  # we default to json
            #    # current_app.logger.debug('sending back json')
            #    mime_type = 'application/json'
            #    output_data = msgconv.extract_values(msg) if msg is not None else None
            #    output_data = json.dumps(output_data)

            response = make_response(output_data, 200)
            response.mimetype = mime_type
            return response

        path = path[:-(len(suffix) + 1)]
        sfx_resp = None
        if suffix == MSG_PATH and path in topics:
            if jsn:
                # TODO : find a better way to interface here...
                sfx_resp = make_response(simplejson.dumps(topics[path].get('msgtype')), 200)
                sfx_resp.mimetype = 'application/json'
            else:
                # broken now, cannot access pyros.rosinterface.topic.get_topic_msg
                # TODO : check if we still need that feature ? JSON first -> maybe not...
                # Or we do it in another way (without needing to import that module)
                #sfx_resp = make_response(definitions.get_topic_msg(topics[path]), 200)
                sfx_resp = make_response('Deprecated feature. use _rosdef instead', 200)
                sfx_resp.mimetype = 'text/plain'
                pass
        elif suffix == SRV_PATH and path in services:
            if jsn:
                sfx_resp = make_response(simplejson.dumps(services[path].get('srvtype'), ignore_nan=True), 200)
                sfx_resp.mimetype = 'application/json'
            else:
                # broken now, cannot access pyros.rosinterface.service.get_service_srv
                # TODO : check if we still need that feature ? JSON first -> maybe not...
                # Or we do it in another way (without needing to import that module)
                #sfx_resp = make_response(definitions.get_service_srv(services[path]), 200)
                sfx_resp = make_response('Deprecated feature. use _rosdef instead', 200)
                sfx_resp.mimetype = 'text/plain'
        elif suffix == CONFIG_PATH:
            if path in services:
                service_name = path

                service = services[service_name]
                dfile = definitions.describe_service(service_name, service, full=full)

                if jsn:
                    sfx_resp = make_response(simplejson.dumps(dfile.tojson(), ignore_nan=True), 200)
                    sfx_resp.mimetype='application/json'
                else:
                    sfx_resp = make_response(dfile.tostring(suppress_formats=True), 200)
                    sfx_resp.mimetype = 'text/plain'
            elif path in topics:
                topic_name = path

                topic = topics[topic_name]
                dfile = definitions.describe_topic(topic_name, topic, full=full)

                if jsn:
                    sfx_resp = make_response(simplejson.dumps(dfile.tojson(), ignore_nan=True), 200)
                    sfx_resp.mimetype ='application/json'
                else:
                    sfx_resp = make_response(dfile.tostring(suppress_formats=True), 200)
                    sfx_resp.mimetype = 'text/plain'
            else:
                sfx_resp = make_response('', 404)
        else:
            current_app.logger.warn('404 : %s', path)
            sfx_resp = make_response('', 404)
        return sfx_resp

    # TODO: think about login rest service before disabling REST services if not logged in
    def post(self, rosname, *args, **kwargs):
        # fail early if no pyros client
        if self.node_client is None:
            current_app.logger.warn('404 : %s', rosname)
            return make_response('', 404)

        try:
            rosname = '/' + rosname
            #current_app.logger.debug('POST')
            length = int(request.environ['CONTENT_LENGTH'])
            use_ros = ('CONTENT_TYPE' in request.environ and
                       ROS_MSG_MIMETYPE == request.environ['CONTENT_TYPE'].split(';')[0].strip())

            services = current_app.config.get("PYROS_SERVICES")
            topics = current_app.config.get("PYROS_TOPICS")
            params = current_app.config.get("PYROS_PARAMS")

            if rosname in services:
                mode = 'service'
                service = rosname
            elif rosname in topics:
                mode = 'topic'
                topic = rosname
            elif rosname in params:
                mode = 'param'
                param = rosname
            elif rosname == '/Robot/ParamDumpAll':
                mode = 'custom'
            elif rosname == '/Robot/ParamLoadAll':
                mode = 'custom'
            elif rosname == '/Robot/ActivateAll':
                mode = 'custom'
            elif rosname == '/Robot/Move':
                mode = 'custom'
            elif rosname == '/Camera/Rotate':
                mode = 'custom'
            elif rosname == '/Robot/LEDControl':
                mode = 'custom'
            else:
                current_app.logger.warn('404 : %s', rosname)
                return make_response('', 404)

            # we are now sending via the client node, which will convert the
            # received dict into the correct message type for the service (or
            # break, if it's wrong.)

            # Trying to parse the input
            try:
                input_data = request.environ['wsgi.input'].read(length)
                input_data = simplejson.loads(input_data or "{}")
                input_data.pop('_format', None)
                #TODO : We get the message structure via the topic, can we already use it for checking before calling rospy ?
            except ValueError as exc_value:
                raise WrongMessageFormat(
                    message="Your request payload was incorrect: {exc_value}".format(exc_value=exc_value),
                    traceback=tblib.Traceback(sys.exc_info()[2]).to_dict()
                )

            # input_msg = input_msg_type() # was topic.rostype but we dont have it here ( cant serialize and transfer easily )
            # current_app.logger.debug('input_msg:%r', input_msg)
            # if use_ros:
            #     input_msg.deserialize(input_data)
            # else:
            #     input_data = json.loads(input_data)
            #     input_data.pop('_format', None)
            #     msgconv.populate_instance(input_data, input_msg)

            response = None
            if mode == 'service':
                current_app.logger.debug('calling service %s with msg : %s', service, input_data)
                ret_msg = self.node_client.service_call(rosname, input_data)
                name, val = input_data.items()[0]

                if use_ros:
                    content_type = ROS_MSG_MIMETYPE
                    output_data = StringIO()
                    ret_msg.serialize(output_data)
                    output_data = output_data.getvalue()
                elif ret_msg:
                    output_data = ret_msg  # the returned message is already converted from ros format by the client
                    output_data['_format'] = 'ros'
                    output_data = simplejson.dumps(output_data, ignore_nan=True)
                    content_type = 'application/json'
                else:
                    output_data = "{}"
                    content_type = 'application/json'

                if ret_msg["flag"] == False:
                    msg = make_dict(description="wrong value, it should be one of enable/disable", result={"error":1})
                else:
                    msg = make_dict(description="succeed", result={"value":val})
                output_data = simplejson.dumps(msg, ignore_nan=True)
                response = make_response(output_data, 200)
                response.mimetype = content_type
            elif mode == 'topic':
                current_app.logger.debug('publishing \n%s to topic %s', input_data, topic)
                self.node_client.topic_inject(rosname, input_data)
                response = make_response('{}', 200)
                response.mimetype = 'application/json'
            elif mode == 'param':
                node_name = rosname.strip('/').split("/")[0]
                dr_client = current_app.dr_dict[node_name] 
                
                system_nodes = current_app.config.get('SYSTEM_PARAM_GROUP')
                name, val = input_data.items()[0]
                rule = find_rule(name, dr_client.get_parameter_descriptions())
                check = True
                msg = None

                if rule != None:
                    defined_type = rule["type"]
                    defined_range = (rule["min"], rule["max"])
                    # Check type
                    if (string_type_check(val) == int and defined_type not in ["int", "double"]) \
                        or (string_type_check(val) == float and defined_type not in ["double"]) \
                        or (string_type_check(val) == str and defined_type not in ["str"]):
                        check = False
                        msg = make_dict(description="wrong type, type should be {:s}".format(defined_type),
                                        result={"error":1})
                    # Check range
                    elif defined_type in ["int", "double"]:
                        val_ = float(val)
                        if val_ > defined_range[1]:
                            msg = make_dict(description="succeed",
                                            warning="given value exceeds the defined range, value should be less than {:f}".format(defined_range[1]),
                                            result={"value":str(defined_range[1])})
                        elif val_ < defined_range[0]:
                            msg = make_dict(description="succeed",
                                            warning="given value exceeds the defined range, value should be greater than {:f}".format(defined_range[0]),
                                            result={"value":str(defined_range[0])})

                else:
                    check = False
                    msg = make_dict(description="not found name", result={"error":1})

                if check:
                    if '/'+node_name in system_nodes:
                        for nname in system_nodes:
                            nname = nname.strip('/').split('/')[0]
                            dr_client = current_app.dr_dict[nname] 
                
                            current_app.logger.debug('setting \n%s %s\'s param %s', input_data, nname, param.get('name'))

                            dr_client.update_configuration(input_data)
                        msg = make_dict(description="succeed", result={"value": val}) if msg == None else msg
                    else:
                        current_app.logger.debug('setting \n%s param %s', input_data, param.get('name'))

                        dr_client.update_configuration(input_data)

                        msg = make_dict(description="succeed", result={"value": val}) if msg == None else msg
                
                output_data = simplejson.dumps(msg, ignore_nan=True)
                response = make_response(output_data, 200)
                response.mimetype = 'application/json'
            elif mode == 'custom':
                if rosname == '/Robot/ActivateAll':
                    def activateAll(input_data):
                        if "request" not in input_data:
                            return make_dict(description="wrong request format",
                                            result={"error":1})

                        val = input_data["request"]
                        for rosService in current_app.config.get('PYROS_SERVICES'):
                            current_app.logger.debug('calling service %s with msg : %s', rosService, input_data)
                            ret_msg = self.node_client.service_call(rosService, input_data)
                            
                            if use_ros:
                                content_type = ROS_MSG_MIMETYPE
                                output_data = StringIO()
                                ret_msg.serialize(output_data)
                            else:
                                content_type = 'application/json'

                            if ret_msg["flag"] == False:
                                return make_dict(description="wrong value, it should be one of enable/disable", result={"error":1})
                        return make_dict(description="succeed", result={"value":val})
                    msg = activateAll(input_data)
                elif rosname == '/Robot/Move':
                    def robotMove(input_data):
                        if "value" not in input_data:
                            return make_dict(description="wrong value format",
                                            result={"error":1})
                        
                        val = input_data['value']
                        if "linear" not in val or "angular" not in val:
                            return make_dict(description="wrong value format",
                                            result={"error":1})
                       
                        linear_val = float(val["linear"])
                        angular_val = float(val["angular"])
                        
                        vel_msg = Twist()
                        vel_msg.linear.x = linear_val
                        vel_msg.linear.y = 0
                        vel_msg.linear.z = 0
                        vel_msg.angular.x = 0 
                        vel_msg.angular.y = 0
                        vel_msg.angular.z = angular_val 
                        pub = current_app.pub['robotMove']
                        pub.publish(vel_msg)

                        return make_dict(description="succeed",
                                        result={"value":val})
                    msg = robotMove(input_data)
                elif rosname == '/Camera/Rotate':
                    def cameraRotate(input_data):
                        if "value" not in input_data:
                            return make_dict(description="wrong value format",
                                            result={"error":1})
                        
                        val = float(input_data['value'])
                        if val > 3.14 or val < -3.14:
                            return make_dict(description="given value exceeds the defined range",
                                            result={"error":1})


                        pub = current_app.pub['cameraRotate']
                        pub.publish(Float64(val))

                        return make_dict(description="succeed",
                                        result={"value":val})
                    msg = cameraRotate(input_data)
                elif rosname == '/Robot/ParamDumpAll':
                    def paramDumpAll(input_data):
                        if "path" not in input_data:
                            return make_dict(description="wrong path format",
                                            result={"error":1})

                        path = input_data["path"]
                        result = {}
                        for rosname, dr_client in current_app.dr_dict.items(): 
                            result[rosname] = dr_client.get_configuration()
                        with open(path, "w") as output_file:
                            output_file.write(yaml.dump(result))
                        return make_dict(description="succeed", result={"value":path})
                    msg = paramDumpAll(input_data)
                elif rosname == '/Robot/ParamLoadAll':
                    def paramLoadAll(input_data):
                        if "path" not in input_data:
                            return make_dict(description="wrong path format",
                                            result={"error":1})
                        path = input_data["path"]
                        with open(path) as input_file:
                            result = yaml.load(input_file.read())
                        for rosname, dr_client in current_app.dr_dict.items(): 
                            dr_client.update_configuration(result[rosname])
                        return make_dict(description="succeed", result={"value":path})
                    msg = paramLoadAll(input_data)
                elif rosname == '/Robot/LEDControl':
                    def ledControl(input_data):
                        if "id" not in input_data or "state" not in input_data:
                            return make_dict(description="wrong request format",
                                            result={"error":1})

                        id = int(input_data['id'])
                        state = int(input_data['state'])

                        if id < 0 or id > 2 or state < 0 or state > 2:
                            return make_dict(description="given value exceeds the defined range",
                                            result={"error":1})

                        import rostopic, roslib

                        data_type = rostopic.get_topic_type('/twinny_robot/LEDControl')[0]
                        LEDControlMsg = roslib.message.get_message_class(data_type)

                        pubmsg =  LEDControlMsg()
                        pubmsg.ID = id
                        pubmsg.STATE = state

                        pub = current_app.pub['LEDControl']
                        pub.publish(pubmsg)

                        return make_dict(description="succeed", result={"value":input_data})
                    msg = ledControl(input_data)

                output_data = simplejson.dumps(msg, ignore_nan=True)
                response = make_response(output_data, 200)
                response.mimetype = 'application/json'
            
            return response

            # converting pyros exceptions to proper rostful exceptions
            # except (InvalidMessageException, NonexistentFieldException, FieldTypeMismatchException) as exc_value:
            #     raise WrongMessageFormat(
            #         message=str(exc_value.message),
            #         traceback=tblib.Traceback(sys.exc_info()[2]).to_dict()
            #     )
            try:
                pass
            except PyrosServiceTimeout as exc_value:
                raise ServiceTimeout(
                    message=str(exc_value.message),
                    traceback=tblib.Traceback(sys.exc_info()[2]).to_dict()
                )
            except PyrosServiceNotFound as exc_value:
                raise ServiceNotFound(
                    message=str(exc_value.message),
                    traceback=tblib.Traceback(sys.exc_info()[2]).to_dict()
                )

        # returning local exceptions
        except WrongMessageFormat as wmf:
            current_app.logger.error('Wrong message format! => {status} \n{exc}'.format(
                status=wmf.status_code,
                exc=wmf.message
            ))
            return make_response(simplejson.dumps(wmf.to_dict(), ignore_nan=True), wmf.status_code)

        except ServiceTimeout as st:
            current_app.logger.error('Service Timeout! => {status} \n{exc}'.format(
                status=st.status_code,
                exc=st.message
            ))
            return make_response(simplejson.dumps(st.to_dict(), ignore_nan=True), st.status_code)

        except ServiceNotFound as snf:
            current_app.logger.error('Service Not Found! => {status} \n{exc}'.format(
                status=snf.status_code,
                exc=snf.message
            ))
            return make_response(simplejson.dumps(snf.to_dict(), ignore_nan=True), snf.status_code)

        # Generic way to return Exceptions we don't know how to handle
        # But we can do a tiny bit better if it s a PyrosException
        except Exception as exc_value:
            exc_type, exc_value, tb = sys.exc_info()
            tb = tblib.Traceback(tb)
            exc_dict = {
                    'exc_type': str(exc_type),
                    # TODO : it would be nice if pyros exception wouldnt need such a check...
                    'exc_value': str(exc_value.message) if isinstance(exc_value, PyrosException) else str(exc_value),
                    'traceback': tb.to_dict()
                 }
            current_app.logger.error('An exception occurred! => 500 \n{exc}'.format(exc=exc_dict))
            return make_response(simplejson.dumps(exc_dict, ignore_nan=True), 500)
            # return make_response(e, 500)

api.add_resource(BackEnd, '/','/<path:rosname>')
# TO have more than json representation : http://stackoverflow.com/a/28520065
