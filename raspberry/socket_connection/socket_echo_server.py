########Human Robot Interaction System #########
#***********************************************************************
## Description:
# Function cretaed in order to be used in the human_interaction.py
# This function creaqtes a connection with a computer in order to onbtain the state of the child in terms of socket
#*********************************************************************
#Author: Ane
#*********************************************************************
# Date: 26/06/20
#*********************************************************************
import socket

host = ''
port = 5560

storedValue = "Yo, what's up?"

reply = ""
 
def setupServer():
    # Create the server
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print("Socket created.")
    try:
        s.bind((host, port))
    except socket.error as msg:
        print(msg)
    print("Socket bind complete.")
    return s

def setupConnection(s):
    #Stablish the connection
    s.listen(1) # Allows one connection at a time.
    conn, address = s.accept()
    print("Connected to: " + address[0] + ":" + str(address[1]))
    return conn

def GET():
    reply = storedValue
    return reply

def REPEAT(dataMessage):
    reply = dataMessage[1]
    return reply

def dataTransfer(conn,s):
        global reply
    # A big loop that sends/receives data until told not to.
    # Receive the data
        data = conn.recv(1024) # receive the data
        data = data.decode('utf-8')
        # Split the data such that you separate the command
        # from the rest of the data.
        dataMessage = data.split(' ', 1)
        command = dataMessage[0]
        if command == 'GET':
            reply = GET()
        elif command == 'REPEAT':# It is used this one. The message sent by the computer is: REPEAT "message"
            reply = REPEAT(dataMessage)
        elif command == 'EXIT':
            print("Our client has left us :(")
        elif command == 'KILL':
            print("Our server is shutting down.")
            s.close()
        else:
            reply = 'Unknown Command'
        # Send the reply back to the client
        conn.sendall(str.encode(reply))
        #print("Data has been sent!")
    
        
