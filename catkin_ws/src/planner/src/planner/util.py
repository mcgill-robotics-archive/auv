import tf

listener = None


def get_listener():
    global listener
    if not listener:
        listener = tf.TransformListener()
    return listener
