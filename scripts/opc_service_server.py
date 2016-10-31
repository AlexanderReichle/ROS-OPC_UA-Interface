#!/usr/bin/env python

import sys
import rospy
import time
sys.path.insert(0, "..")

from opcua import Client
from opcua import ua
from opc_service.srv import *
from distutils.util import strtobool

# Client with IP
# GESI-Server Global
client = Client("opc.tcp://141.58.122.40:4841")
# GESI-Server Local
#client = Client("opc.tcp://192.168.1.10:4841")

def handle_request(req):
    try:
        #connect to client
        client.connect()        
    except:
        return OPCMenuResponse("Error could not connect to server.", "-201x000")
    
    # Set Root-Node
    root = client.get_root_node()

    # read
    if req.option == 'read':
        if req.cps != "":
            # Get the Variable by Browsepath
            myvar = root.get_child(["0:Objects", "2:GESI", "2:ARENA", "2:%s" % (req.cps), "2:%s" % (req.function), "2:%s" % (req.variable)])
        elif req.cps == "":
            # Get the Variable by NodeId
            myvar = client.get_node("ns=2;%s" % req.variable)
            #Example: myvar = client.get_node("ns=2;s=15892.local")
        #print myvar.get_data_value()
        responseValue = str(myvar.get_value())
        responseAnswer = "Read Suceeded"
    
    # write
    elif req.option == 'write':
        if req.cps != "":
            # Get the Variable by Browsepath
            myvar = root.get_child(["0:Objects", "2:GESI", "2:ARENA", "2:%s" % (req.cps), "2:%s" % (req.function), "2:%s" % (req.variable)])
        elif req.cps == "":
            # Get the Variable by NodeId
            myvar = client.get_node("ns=2;%s" % req.variable)
        # get Value to write
        inputValue = req.valueToWrite
        # get expected Datatype
        dataType = str(myvar.get_data_value())
        # Write Boolean
        if dataType.find('Boolean') > 0:
            myvar.set_value(ua.Variant(strtobool(inputValue), ua.VariantType.Boolean))
        # Write Int32    
        if dataType.find('Int') > 0:
            myvar.set_value(ua.Variant(int(inputValue), ua.VariantType.UInt32))
        # Write Double    
        if dataType.find('Double') > 0:
            myvar.set_value(ua.Variant(float(inputValue), ua.VariantType.Double))
        # Write String    
        if dataType.find('String') > 0:
            myvar.set_value(ua.Variant(str(inputValue), ua.VariantType.String))
        # Set ROS Response               
        responseValue =  str(myvar.get_value())
        responseAnswer = "Write Suceeded"  
    
    # subscribe
    elif req.option == 'subscribe':
        if req.cps != "":
            # Get the Variable by Browsepath
            myvar = root.get_child(["0:Objects", "2:GESI", "2:ARENA", "2:%s" % (req.cps), "2:%s" % (req.function), "2:%s" % (req.variable)])
        elif req.cps == "":
            # Get the Variable by NodeId
            myvar = client.get_node("ns=2;%s" % req.variable)
            #Example: myvar = client.get_node("ns=2;s=15892.local")
        initialValue = str(myvar.get_value())
        actualValue = initialValue
        # Read Value until the Value changes its status
        while initialValue == actualValue:
            actualValue = str(myvar.get_value())
        responseValue = actualValue
        responseAnswer = "Read Suceeded"

    # call Method
    elif req.option == 'callMethod':
        # Get the Object by Browsepath
        obj = root.get_child(["0:Objects", "2:GESI", "2:ARENA", "2:%s" % (req.cps)])
        # Get the Object by NodeId
        #obj = client.get_node("ns=2;s=15892.local")
        responseValue = obj.call_method("2:%s" % (req.function), "%s" % (req.variable))
        responseAnswer = "Method Call Suceeded"

    # call addPackage GESI-only
    elif req.option == 'addPackage':
        cps = str(req.cps)
        if cps.find("=") < 0:
            # Get the Object by Browsepath
            obj = root.get_child(["0:Objects", "2:GESI", "2:ARENA", "2:%s" % (req.cps)])
        elif cps.find("=") > 0:
            # Get the Object by NodeId
            obj = client.get_node("ns=2;%s" % (req.cps))   
        responseValue = obj.call_method("2:addPackage", "%s" % (req.function))
        responseAnswer = "Call addPackage Suceeded"    

    # call remPackage GESI-only
    elif req.option == 'remPackage':
        cps = str(req.cps)
        if cps.find("=") < 0:
            # Get the Object by Browsepath
            obj = root.get_child(["0:Objects", "2:GESI", "2:ARENA", "2:%s" % (req.cps)])
        elif cps.find("=") > 0:
            # Get the Object by NodeId
            obj = client.get_node("ns=2;%s" % (req.cps)) 
        function = str(req.function)
        # get Function in the right format
        function = function[2:]
        responseValue = obj.call_method("2:remPackage", "%s" % (function))
        responseAnswer = "Call remPackage Suceeded"     

    # login with __register()
    elif req.option == 'login':
        obj = root.get_child(["0:Objects", "2:GESI", "2:ARENA"])
        cps = str(req.cps)
        # get NodeId in the right format
        CPS_Id = str(cps[2:])
        # get all Nodes(CPS) that are registrated at the Server
        childNodes = str(obj.get_children())
        # CPS not found
        if childNodes.find(CPS_Id) < 0:
            # Get the Object by Browsepath
            obj = root.get_child(["0:Objects", "2:GESI", "2:ARENA"])
            # Get the Object by NodeId
            #obj = client.get_node("ns=2;s=15892.local")
            responseValue = obj.call_method("2:addCPS", "%s" % (req.displayName))
            responseAnswer = "New CPS Has Been Created. Login Successful"  
        # CPS found     
        elif childNodes.find(CPS_Id) > 0:
            responseValue = CPS_Id
            responseAnswer = "CPS Already Exists. Login Successful"   

    # logout
    elif req.option == 'logout':
        cps = str(req.cps)
        if cps.find("=") < 0:
            # Get the Object by Browsepath
            obj = root.get_child(["0:Objects", "2:GESI", "2:ARENA", "2:%s" % (req.cps)])
        elif cps.find("=") > 0:
            # Get the Object by NodeId
            obj = client.get_node("ns=2;%s" % (req.cps)) 
        # get all registrated Functions    
        childNodes = str(obj.get_children())
        childNodesList = childNodes.split()
        # extract Functions from Variables
        for i in childNodesList:
            if "." in i:
                pass
            else:
                # delet all Functions
                startIndex = i.find(";s=")
                NodeId = str(i[startIndex+3:startIndex+8])   
                obj.call_method("2:remPackage", NodeId)
        responseValue = ""
        responseAnswer = "Logout Successful"
                    
    
    # Not known option
    else:
        responseValue = ""
        responseAnswer = "Option currently is not supported"    
    
    # disconnect from client
    client.disconnect()
    # Send ROS Response
    return OPCMenuResponse(responseAnswer, responseValue)

# Initialize the ROS Service
def opc_service_server():
    rospy.init_node('opc_service_server')
    s = rospy.Service('opc_service', OPCMenu, handle_request)
    print "Ready to receive OPC commands."
    rospy.spin()

#def __register(object, CPS_Id, childNodes,root):    


if __name__ == "__main__":
    opc_service_server()

