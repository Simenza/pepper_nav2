#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RequestCreator(Node):
    def __init__(self):
        super().__init__('request_creator')
        self.publisher = self.create_publisher(String, '/goal_request', 10)
        self.subscription = self.create_subscription(
            String,
            '/selected_recipe',
            self.recipe_callback,
            10)
        
        self.recipes = ["pasta_al_pomodoro", "polpette_al_sugo"]
        self.ingredients = {
            "pasta_al_pomodoro": ["pasta", "pomodoro", "formaggio"],
            "polpette_al_sugo": ["macinato", "pomodoro", "pangrattato"]
        }
        self.places = {
            "dispensa": ["pasta", "pomodoro", "pangrattato"],
            "frigorifero": ["macinato", "formaggio"]
        }
    
    def recipe_callback(self, msg):
        self.recipe_input = msg.data.strip()
        self.get_logger().info(f"Ricetta ricevuta da interfaccia: {self.recipe_input}")
        self.process_request()

    def process_request(self):
        if self.recipe_input not in self.recipes:
            self.get_logger().error(f"Ricetta '{self.recipe_input}' non disponibile.")
            return

        ingredienti_necessari = self.ingredients[self.recipe_input]
        luoghi_unici = []

        # Determina i luoghi da visitare in base agli ingredienti
        for ingrediente in ingredienti_necessari:
            for luogo, contenuto in self.places.items():
                if ingrediente in contenuto:
                    luoghi_unici.append(luogo)
                    break  # Un ingrediente è in un solo luogo

        # Inserisce 'tavolo' dopo ogni luogo
        luoghi_effettivi = []
        for luogo in luoghi_unici:
            luoghi_effettivi.append(luogo)
            luoghi_effettivi.append("tavolo")
        luoghi_effettivi.append("base")

        self.get_logger().info(f"Andrò in: {luoghi_effettivi}")

        self.luoghi_da_visitare = luoghi_effettivi
        self.current_index = 0
        self.timer = self.create_timer(2.0, self.timer_callback)

    def timer_callback(self):
        if self.current_index >= len(self.luoghi_da_visitare):
            self.get_logger().info("Tutti i goal pubblicati.")
            self.timer.cancel()
            return

        luogo = self.luoghi_da_visitare[self.current_index]
        msg = String()
        msg.data = luogo
        self.publisher.publish(msg)
        self.get_logger().info(f"Pubblicato goal: {luogo}")
        self.current_index += 1

def main(args=None):
    rclpy.init(args=args)
    node = RequestCreator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
