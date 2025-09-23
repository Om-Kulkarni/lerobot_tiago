#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@file tiago_host.py
@brief Network host script for the Tiago robot.
@details This script runs on the Tiago's onboard computer. It instantiates the Tiago hardware
         driver, listens for a client connection, and then enters a loop to receive actions
         and send back observations.
"""

import socket
import struct
import pickle
import rospy # type: ignore
from tiago import Tiago   # type: ignore

# Socket configuration
HOST = '0.0.0.0'  # Listen on all available network interfaces
PORT = 65432

def recv_msg(conn):
    """Helper function to receive a message with a 4-byte length prefix."""
    # Read the header to get the message length
    raw_msglen = conn.recv(4)
    if not raw_msglen:
        return None
    msglen = struct.unpack('>I', raw_msglen)[0]

    # Read the full message payload
    data = bytearray()
    while len(data) < msglen:
        packet = conn.recv(msglen - len(data))
        if not packet:
            return None
        data.extend(packet)
    return data

def send_msg(sock, msg):
    """Helper function to send a message with a 4-byte length prefix."""
    # Prefix payload with its length as a 4-byte unsigned integer (network byte order)
    payload = pickle.dumps(msg, protocol=2) # protocol=2 for Python 2 compatibility
    header = struct.pack('>I', len(payload))
    sock.sendall(header + payload)

def main():
    """Main function to run the server."""
    rospy.init_node('tiago_socket_host', anonymous=True)

    # 1. Instantiate the hardware driver
    robot = Tiago()

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        server_socket.bind((HOST, PORT))
        server_socket.listen(1)
        rospy.loginfo("Tiago Host listening for client on {}:{}".format(HOST, PORT))

        while not rospy.is_shutdown():
            try:
                rospy.loginfo("Waiting for a client connection...")
                conn, addr = server_socket.accept()
                rospy.loginfo("Connected by {}".format(addr))

                try:
                    while not rospy.is_shutdown():
                        # 2. Receive a message from the client
                        message_data = recv_msg(conn)
                        if not message_data:
                            rospy.logwarn("Client disconnected.")
                            break
                        
                        message = pickle.loads(message_data)

                        # Check if the message is a calibration command or a regular action
                        if isinstance(message, str) and message == 'calibrate':
                            rospy.loginfo("Calibration command received. Moving to home position.")
                            robot.go_to_home_position()
                            # Send a simple confirmation back to the client
                            send_msg(conn, 'calibration_complete')
                            rospy.loginfo("Calibration complete. Confirmation sent.")
                        
                        elif isinstance(message, dict):
                            # It's a regular action, proceed as before
                            action = message
                            robot.send_action(action)
                            observation = robot.get_observation()
                            send_msg(conn, observation)
                        
                        else:
                            rospy.logwarn("Received unknown message format from client.")

                except (socket.error, pickle.UnpicklingError) as e:
                    rospy.logerr("An error occurred during communication: {}".format(e))
                finally:
                    robot.stop_robot()
                    conn.close()
                    rospy.loginfo("Connection closed.")

            except socket.error as e:
                rospy.logerr("Socket error while waiting for connection: {}".format(e))
                rospy.sleep(1.0) # Wait before trying again
            
    except Exception as e:
        rospy.logerr("An unexpected server error occurred: {}".format(e))
    finally:
        server_socket.close()
        rospy.loginfo("Server socket closed.")

if __name__ == "__main__":
    main()



                    