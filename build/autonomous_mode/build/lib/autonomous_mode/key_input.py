import rclpy
from pynput.keyboard import Key,Listener
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import threading 
import sys
import subprocess
from tutorial_interfaces.msg import Board
import time
from rclpy.executors import MultiThreadedExecutor

key_state={
            "key_up":0,
            "key_down":0,
            "key_left":0,
            "key_right":0,
            "key_left_shift":0,
            "key_left_ctrl":0,
            "w":0,
            "a":0,
            "s":0,
            "d":0,
            "q":0,
            "e":0,
            "c":0,
            "z":0,
            "x":0,
            "j":0
        }

key_values=Board()

class keyboard_reader(Node):

    def __init__(self) -> None:
        super().__init__('key_reader')
        print('lund')
        with Listener(
        on_press=self.on,
        on_release=self.off) as listener:
            listener.join()
        
        # self.key_publisher = self.create_publisher(Board, '/keyboard',qos_profile)
    

    # def assign(self):
       
    #     self.key_values.key_up=self.key_state["key_up"]
    #     self.key_values.key_down=self.key_state["key_down"]
    #     self.key_values.key_left=self.key_state["key_left"]
    #     self.key_values.key_right=self.key_state["key_right"]
    #     self.key_values.key_left_shift=self.key_state["key_left_shift"]
    #     self.key_values.key_left_ctrl=self.key_state["key_left_ctrl"]
    #     self.key_values.w=self.key_state['w']
    #     self.key_values.a=self.key_state['a']
    #     self.key_values.s=self.key_state['s']
    #     self.key_values.d=self.key_state['d']
    #     self.key_values.q=self.key_state['q']
    #     self.key_values.e=self.key_state['e']
    #     self.key_values.c=self.key_state['c']
    #     self.key_values.z=self.key_state['z']
    #     self.key_values.x=self.key_state['x']
    #     self.key_values.j=self.key_state['j']
    #     self.key_publisher.publish(self.key_values)
    #     time.sleep(0.005)


    


    def on(self,key:Key):
        key =str(key)
        print(key)
        if(key=="Key.esc"):
            print("Exiting")
            subprocess.run(["stty","echo"]) # password mode
            sys.exit() # to the parent thread
        if(key=="Key.up"):
            key_state["key_up"]=1
        if(key=="Key.down"):
            key_state["key_down"]=1
        if(key=="Key.left"):
            key_state["key_left"]=1
        if(key=="Key.right"):
            key_state["key_right"]=1
        if(key=="Key.shift"):
            key_state["key_left_shift"]=1
        if(key=="Key.ctrl"):
            key_state["key_left_ctrl"]=1
        if(key=="'w'"):
            key_state["w"]=1
        if(key=="'a'"):
            key_state["a"]=1
        if(key=="'s'"):
            key_state["s"]=1
        if(key=="'d'"):
            key_state["d"]=1
        if(key=="'q'"):
            key_state["q"]=1
        if(key=="'e'"):
            key_state["e"]=1
        if(key=="'c'"):
            key_state["c"]=1
        if(key=="'z'"):
            key_state["z"]=1
        if(key=="'x'"):
            key_state["x"]=1
        if(key=="'j'"):
            key_state["j"]=1

    def off(self,key:Key):
        key = str(key)
        if(key=="Key.up"):
            key_state["key_up"]=0
        if(key=="Key.down"):
            key_state["key_down"]=0
        if(key=="Key.left"):
            key_state["key_left"]=0
        if(key=="Key.right"):
            key_state["key_right"]=0
        if(key=="Key.shift"):
            key_state["key_left_shift"]=0
        if(key=="Key.ctrl"):
            key_state["key_left_ctrl"]=0
        if(key=="'w'"):
            key_state["w"]=0
        if(key=="'a'"):
            key_state["a"]=0
        if(key=="'s'"):
            key_state["s"]=0
        if(key=="'d'"):
            key_state["d"]=0
        if(key=="'q'"):
            key_state["q"]=0
        if(key=="'e'"):
            key_state["e"]=0
        if(key=="'c'"):
            key_state["c"]=0
        if(key=="'z'"):
            key_state["z"]=0
        if(key=="'x'"):
            key_state["x"]=0
        if(key=="'j'"):
            key_state["j"]=0

class keyboard_publisher(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('key_publisher')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_ALL,
        )
        self.key_pub=self.create_publisher(Board,'/keyboard',qos_profile)
        threading.Thread(target=self.publisher_callback).start()

    def publisher_callback(self):
        while True:
            print("d")
            key_values.key_up=key_state["key_up"]
            key_values.key_down=key_state["key_down"]
            key_values.key_left=key_state["key_left"]
            key_values.key_right=key_state["key_right"]
            key_values.key_left_shift=key_state["key_left_shift"]
            key_values.key_left_ctrl=key_state["key_left_ctrl"]
            key_values.w=key_state['w']
            key_values.a=key_state['a']
            key_values.s=key_state['s']
            key_values.d=key_state['d']
            key_values.q=key_state['q']
            key_values.e=key_state['e']
            key_values.c=key_state['c']
            key_values.z=key_state['z']
            key_values.x=key_state['x']
            key_values.j=key_state['j']
            self.key_pub.publish(key_values)
            time.sleep(0.5)




    
def main(args=None) -> None:
    
    rclpy.init(args=args)
    print("Enter the key here, use ctrl c to exit,press esc to exit")
    subprocess.run(["stty","-echo"])

    try:

        key_publisher =keyboard_publisher ()
        key_reader = keyboard_reader()

        executor = MultiThreadedExecutor()
        executor.add_node(key_publisher)
        executor.add_node(key_reader)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            key_publisher.destroy_node()
            key_reader.destroy_node()

    finally:
        rclpy.shutdown()

    
    
    


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)