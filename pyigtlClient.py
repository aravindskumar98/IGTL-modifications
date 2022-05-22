import pyigtl
client = pyigtl.OpenIGTLinkClient("127.0.0.1", 18944)

while True:
    print("waiting for message.....")
    message = client.wait_for_message("ParamEventHandler")
    print(message)# send the message.  At that point, the Omni will try to stay in place
