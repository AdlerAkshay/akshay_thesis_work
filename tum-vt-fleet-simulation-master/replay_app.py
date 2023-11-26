import os
import sys
from flask import Flask, request
from flask_socketio import SocketIO
from flask_cors import CORS

from src.ReplayFromResult import Replay


"""
Create a flask app, an a web-socket to communicate with the frontend.
Use eventlet to allow threading.
Create only one Replay object.
"""
app = Flask(__name__)
socket_io = SocketIO(app, cors_allowed_origins='*', async_mode='eventlet')
CORS(app)
replay = Replay()


@socket_io.on('connect', namespace='/dd')
def ws_conn():
    """
    Once a new client is connected, they should be informed about the network.
    If a replay is not yet started, run it providing a web-socket.
    """
    socket_io.emit('network', (replay.get_gj_network_info()), namespace="/dd", room=request.sid)

    if not replay.started:
        replay.start(socket_io)

# TODO # write listener as interface to Replay (change time, layers, exit, ...)


@socket_io.on('disconnect', namespace='/dd')
def ws_disconn():
    # TODO # pause replay
    pass


if __name__ == "__main__":
    """
    Open a webscoket connection and load scenario.
    """
    # init output dir from sys.argv here
    if len(sys.argv) == 2:
        tmp = sys.argv[1]
        if os.path.isdir(tmp):
            output_dir = tmp
    else:
        # TODO # after bugfix is done: throw error here
        output_dir = r"studies/Munich_Pia/pia_d_50_v_20_s_0"
    replay.load_scenario(output_dir)
    socket_io.run(app, "localhost", port=5000)
