from random import randint
from time import sleep
from start_just_robot import main
from multiprocessing import Lock, Process, Value, Array
from ctypes import Structure, c_int, c_bool


class Interpolator():
    """
        The interpolator gets lists of desidered movements and the time to get there
        Its job is to interpolate the sequence of small movements for a smooth transition
        It is also responsible for the idle movements to keep the robot "alive"
    """
    class _Movement(Structure):
        _fields_ = [("tower_1", c_int), ("tower_2", c_int), ("tower_3", c_int), ("base", c_int), ("ears", c_int)]

        def __eq__(self, other):
            for fld in self._fields_:
                if getattr(self, fld[0]) != getattr(other, fld[0]):
                    return False
            return True

    NONE_MOVEMENT = _Movement(0, 0, 0, 0, 0)
    EMISSION_RATE = 0.1  # how many seconds to wait between an emission and another
    QUEUE_SIZE = 10  # size of the movement queue
    IDLE_MOVE_INTENSITY = 20  # how much the robot can move when initiating an idle move
    IDLE_MOVE_FREQUENCY = 20  # how many empty emissions before it does an idle move?

    def __init__(self):
        self.movements_queue = Array(Interpolator._Movement, [Interpolator.NONE_MOVEMENT for _ in range(Interpolator.QUEUE_SIZE)])
        self.queue_index = Value(c_int, 0)
        self.stop_all = Value(c_bool, False)
        self.lock = Lock()
        self.sender = Process(
            target=Interpolator.sender_thread,
            args=(self.movements_queue, self.queue_index, self.stop_all, self.lock)
        )
        self.sender.start()

    def stop(self):
        self.stop_all.value = True
        self.sender.join()

    # FIXME: not really good, may consider a contextmanager
    def __del__(self):
        self.stop()

    def interpolate_sequence(self, sequence):
        """
        _summary_

        Args:
            sequence (list<motor_dict, duration>): sorted list of movements to do alongside their duration.
                                                   The movements are ABSOLUTE (atm)
        """
        # TODO: new case, assume motor_dict does not include all the motors

        TIME_STEPS_IN_SECOND = int(1 / Interpolator.EMISSION_RATE)  # 1s/0.100s = 10 calls to the driver per second

        # TODO: behaviour with duration=0?
        for (movement, duration) in sequence:
            total_time_steps = duration*TIME_STEPS_IN_SECOND
            # absolute_movement = {key: relative+absolute for (key, relative), (_, absolute) in zip(movement.items(), robot.believed_motor_pos.items())}
            for i in range(1, total_time_steps+1):
                intermidiate_pos = {key: int(value/total_time_steps*i) for key, value in movement.items()}

                self.lock.acquire()
                next_index = None
                while next_index is None and not self.stop_all.value:
                    for i in range(Interpolator.QUEUE_SIZE):
                        if self.movements_queue[(self.queue_index.value+i) % Interpolator.QUEUE_SIZE] == Interpolator.NONE_MOVEMENT:
                            next_index = (self.queue_index.value+i) % Interpolator.QUEUE_SIZE
                            break
                    if next_index is None:
                        self.lock.release()
                        sleep(Interpolator.EMISSION_RATE)
                        self.lock.acquire()
                if next_index is not None:
                    self.movements_queue[next_index] = Interpolator._Movement(**intermidiate_pos)
                self.lock.release()

    @staticmethod
    def sender_thread(movements_queue, queue_index, stop, lock):
        # FIXME: replace with ROS
        robot = main()
        robot.reset_position()
        last_position = robot.reset_pos  # FIXME

        idle_count = 0  # how long it has been idle
        while not stop.value:
            with lock:
                next_mov = movements_queue[queue_index.value]

                if next_mov == Interpolator.NONE_MOVEMENT:
                    idle_count = (idle_count+1) % Interpolator.IDLE_MOVE_FREQUENCY

                    if idle_count == 0:
                        # perform some idle movement
                        next_mov = {
                                        motor: pos+randint(
                                            -Interpolator.IDLE_MOVE_INTENSITY,
                                            Interpolator.IDLE_MOVE_INTENSITY
                                        ) for motor, pos in last_position.items()
                                    }  # idle movement
                else:
                    next_mov = {
                                    'tower_1': next_mov.tower_1,
                                    'tower_2': next_mov.tower_2,
                                    'tower_3': next_mov.tower_3,
                                    'base': next_mov.base,
                                    'ears': next_mov.ears
                                }
                    last_position = next_mov

                # FIXME: replace with ROS
                robot.goto_position(next_mov, delay=Interpolator.EMISSION_RATE, wait=False)

                # circular queue, select next index
                movements_queue[queue_index.value] = Interpolator.NONE_MOVEMENT
                queue_index.value = (queue_index.value+1) % Interpolator.QUEUE_SIZE

            sleep(Interpolator.EMISSION_RATE)  # FIXME: probably not good


# test the code
if __name__ == "__main__":
    interpolator = Interpolator()
    sleep(20)  # test idle movement for 20s

    # try a sequence
    goal_position = {
        'tower_1': 100,
        'tower_2': 50,
        'tower_3': 50,
        'base': 100,
        'ears': 100
    }
    time_to_get_there = 3  # seconds
    sequence = [(goal_position, time_to_get_there)]
    interpolator.interpolate_sequence(sequence)

    sleep(5)  # test idle movement again
    interpolator.stop()
