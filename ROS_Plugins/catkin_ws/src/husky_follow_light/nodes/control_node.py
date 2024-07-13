import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ControlNode:

    def __init__(self):
        rospy.init_node('control_node', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/husky_model/husky/camera', Image, self.image_callback)
        self.cmd_pub = rospy.Publisher('/husky_model/husky/cmd_vel', Twist, queue_size=10)
        self.move_duration = 1  # Tempo di movimento in avanti ridotto
        self.process_frequency = 20  
        self.last_processed_time = rospy.Time.now()
        self.image_received = False  # Flag per indicare se è stata ricevuta un'immagine attuale
        self.image_data = None  # Variabile per memorizzare l'ultima immagine ricevuta
        

    def image_callback(self, data):
        # Gestione della frequenza di elaborazione delle immagini
        if rospy.Time.now() - self.last_processed_time < rospy.Duration(1.0 / self.process_frequency):
            return
        self.last_processed_time = rospy.Time.now()

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.image_data = cv_image
            self.image_received = True
        except CvBridgeError as e:
            rospy.logerr(e)
            return

    def start_image_processing(self):
        # Attiva il subscriber per ricevere immagini
        self.image_sub = rospy.Subscriber('/husky_model/husky/camera', Image, self.image_callback)

    def stop_image_processing(self):
        # Sospendi il subscriber per non ricevere più immagini
        self.image_sub.unregister()
        self.image_received = False
        self.image_data = None

    def make_decision_based_on_image(self):
        if not self.image_received:
            rospy.logwarn("Nessuna immagine attuale disponibile per prendere decisioni.")
            return

        dominant_sector, value = self.find_dominant_sector(self.image_data)
        print(dominant_sector)

        if value == 0 or dominant_sector == 'left':
                # Gira a destra di 90° se il valore era già 0 in precedenza
                self.turn_left(30)
        elif dominant_sector == 'right':
            self.turn_right(30)
        else:
            self.move_on()

        # Dopo aver preso la decisione, puoi interrompere la ricezione delle immagini
        self.stop_image_processing()



    def move_on(self):
        rate = rospy.Rate(10)
        
        print("On")
        twist = Twist()
        twist.linear.x = 0.7
        twist.angular.z = 0.0 
    
        duration = self.move_duration

        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < rospy.Duration(duration):
            self.cmd_pub.publish(twist)  # Pubblica il messaggio Twist
            rate.sleep()
        
        # Ferma il movimento angolare dopo aver ruotato
        twist.linear.x = 0.0
        self.cmd_pub.publish(twist)  

    def turn_left(self, angle):
        rate = rospy.Rate(10)
        print("Left")
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 1.2  # Velocità angolare positiva per ruotare a sinistra

        # Converti l'angolo da gradi a radianti
        angle_radians = np.deg2rad(angle)
        
        # Calcola la durata necessaria
        duration = angle_radians / twist.angular.z

        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < rospy.Duration(duration):
            self.cmd_pub.publish(twist)  # Pubblica il messaggio Twist
            rate.sleep()
        
        # Ferma il movimento angolare dopo aver ruotato
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def turn_right(self, angle):
        rate = rospy.Rate(10)
        print("Right")
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -1.2
        # Converti l'angolo da gradi a radianti
        angle_radians = np.deg2rad(angle)
        
        # Calcola la durata necessaria
        duration = angle_radians / abs(twist.angular.z)

        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time < rospy.Duration(duration):
            self.cmd_pub.publish(twist)  # Pubblica il messaggio Twist
            rate.sleep()
        
        # Ferma il movimento angolare dopo aver ruotato
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)  



    def find_dominant_sector(self, cv_image):
        # Assicurati che l'immagine sia stata caricata correttamente
        if cv_image is None:
            rospy.logerr("Errore nel caricamento dell'immagine.")
            exit()

        # Calcola le dimensioni dell'immagine
        height, width, _ = cv_image.shape

        # Definisci i limiti per i tre settori (sinistra, centro e destra)
        left_limit = width * 2 // 5
        center_limit = width * 3 // 5

        # Divide l'immagine nei tre settori
        left_region = cv_image[:, :left_limit, :]
        center_region = cv_image[:, left_limit:center_limit, :]
        right_region = cv_image[:, center_limit:, :]

        # Conta i pixel rossi in ciascun settore
        red_pixels_left = self.count_red_pixels(left_region)
        red_pixels_center = self.count_red_pixels(center_region)
        red_pixels_right = self.count_red_pixels(right_region)

        # print(red_pixels_left, red_pixels_center, red_pixels_right)

        # Determina il settore con la maggioranza di pixel rossi
        sectors = {'left': red_pixels_left, 'center': red_pixels_center, 'right': red_pixels_right}

        # Verifica se tutti i settori hanno più di 3000 pixel rossi
        if red_pixels_left > 1000 and red_pixels_center > 1000 and red_pixels_right > 1000:
            dominant_sector = 'center'
            max_value = red_pixels_center
        else:
            # Trova il settore con il massimo numero di pixel rossi
            dominant_sector = max(sectors, key=sectors.get)
            max_value = sectors[dominant_sector]

            if max_value < 50:
                # Trova il secondo valore più alto
                sorted_sectors = sorted(sectors.values(), reverse=True)
                second_max_value = sorted_sectors[1]

                if max_value < second_max_value + 10:
                    max_value = 0
                    dominant_sector = 'none'  # Puoi impostare su 'none' o un altro valore predefinito
            else:
                max_value = sectors[dominant_sector]

        return dominant_sector, max_value



    def count_red_pixels(self, image):
        # Converti l'immagine da RGB a HSV
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Definisci i range di colori rossi in HSV
        lower_red1 = np.array([0, 90, 0])    # Valori minimi di tonalità, saturazione e valore per il rosso
        upper_red1 = np.array([10, 255, 50])  # Valori massimi di tonalità, saturazione e valore per il rosso

        lower_red2 = np.array([170, 90, 0])  # Utilizzo del range corretto per OpenCV
        upper_red2 = np.array([179, 255, 50])

        # Crea una maschera per isolare i pixel rossi nell'immagine HSV
        mask_red1 = cv2.inRange(image_hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(image_hsv, lower_red2, upper_red2)
        mask_red = mask_red1 | mask_red2
        # Conta i pixel rossi usando la maschera
        red_pixels = np.count_nonzero(mask_red)
        
        return red_pixels


if __name__ == '__main__':
    try:
        cn = ControlNode()
        cn.start_image_processing()  # Inizia a ricevere immagini
        rate = rospy.Rate(1)  # Frequenza di 1 Hz per esempio
        while not rospy.is_shutdown():
            cn.make_decision_based_on_image()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
