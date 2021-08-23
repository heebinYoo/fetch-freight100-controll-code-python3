import time


def busy_delay(sec):
    start = time.time()
    while True:
        if time.time() > start + sec:
            break
    return True
