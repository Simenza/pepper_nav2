import os
from flask import Flask, render_template, request, jsonify, redirect, url_for
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import threading

# 🔧 Percorso assoluto alla cartella dove si trova server.py
base_path = os.path.dirname(os.path.abspath(__file__))

app = Flask(
    __name__,
    template_folder=os.path.join(base_path, 'templates'),
    static_folder=os.path.join(base_path, 'static')
)
rclpy.init()

class RecipeBridge(Node):
    def __init__(self):
        super().__init__('recipe_bridge')
        self.publisher = self.create_publisher(String, '/selected_recipe', 10)
        self.finished = False  
        self.create_subscription(Bool, '/goals_completed', self.goals_completed_callback, 10)

    def send_recipe(self, recipe_name):
        self.finished = False
        msg = String()
        msg.data = recipe_name
        self.publisher.publish(msg)
        self.get_logger().info(f"Pubblicata ricetta: {recipe_name}")

    def goals_completed_callback(self, msg):
        if msg.data:
            self.get_logger().info("Robot ha finito tutti i goal.")
            self.finished = True

bridge = RecipeBridge()
bridge.finished = True

@app.route('/')
def home():
    if bridge.finished:
        recipes = [
            {"name": "pasta_al_pomodoro", "image": "pasta_pomodoro.jpg"},
            {"name": "polpette_al_sugo", "image": "polpette_sugo.jpg"}
        ]
        return render_template('index.html', recipes=recipes)
    else:
        return render_template('waiting.html')

@app.route('/select', methods=['POST'])
def select():
    recipe = request.form['recipe']
    bridge.send_recipe(recipe)
    bridge.finished = False  
    return redirect(url_for('home')) 

@app.route('/check_finished')
def check_finished():
    return jsonify({'finished': bridge.finished})

def ros_spin():
    rclpy.spin(bridge)

# Avvio ROS in un thread separato
threading.Thread(target=ros_spin, daemon=True).start()

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
