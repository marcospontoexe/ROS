#!/usr/bin/env python

import rospy
from your_package_name.srv import MyServiceMessage, MyServiceMessageResponse  # Substitua 'your_package_name' pelo nome do seu pacote
import os

class SpotRecorder:
    def __init__(self):
        self.spots = []
        self.service = rospy.Service('/save_spot', MyServiceMessage, self.handle_save_spot)

    def handle_save_spot(self, req):
        if req.label == "end":
            return self.save_to_file()
        else:
            return self.record_spot(req.label)

    def record_spot(self, label):
        # Aqui você deve substituir pelos métodos reais para obter a posição e orientação do robô
        current_position = "x:0, y:0, z:0"  # Exemplo de posição
        current_orientation = "x:0, y:0, z:0, w:1"  # Exemplo de orientação
        self.spots.append({'label': label, 'position': current_position, 'orientation': current_orientation})
        return MyServiceMessageResponse(navigation_successful=True, message="Spot recorded successfully")

    def save_to_file(self):
        try:
            with open('spots.txt', 'w') as file:
                for spot in self.spots:
                    file.write(f"Label: {spot['label']}, Position: {spot['position']}, Orientation: {spot['orientation']}\n")
            return MyServiceMessageResponse(navigation_successful=True, message="File saved successfully")
        except Exception as e:
            return MyServiceMessageResponse(navigation_successful=False, message=f"Failed to save file: {str(e)}")

if __name__ == "__main__":
    rospy.init_node('spot_recorder')
    spot_recorder = SpotRecorder()
    rospy.spin()
