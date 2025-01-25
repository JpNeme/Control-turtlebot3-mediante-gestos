import cv2
import mediapipe as mp
import rospy
from std_msgs.msg import Int32

# Inicializamos MediaPipe para pose y manos
mp_drawing = mp.solutions.drawing_utils  # Herramientas para dibujar landmarks
mp_drawing_styles = mp.solutions.drawing_styles  # Estilos de dibujo
mp_pose = mp.solutions.pose  # Solución de pose
mp_hands = mp.solutions.hands  # Solución de manos

# Definimos los landmarks de los dedos índice, corazón, anular y meñique
index_pip = mp_hands.HandLandmark.INDEX_FINGER_PIP
index_tip = mp_hands.HandLandmark.INDEX_FINGER_TIP
middle_pip = mp_hands.HandLandmark.MIDDLE_FINGER_PIP
middle_tip = mp_hands.HandLandmark.MIDDLE_FINGER_TIP
ring_pip = mp_hands.HandLandmark.RING_FINGER_PIP
ring_tip = mp_hands.HandLandmark.RING_FINGER_TIP
pinky_pip = mp_hands.HandLandmark.PINKY_PIP
pinky_tip = mp_hands.HandLandmark.PINKY_TIP

# Lista de puntos de la cara que queremos excluir
EXCLUDE_FACE_LANDMARKS = [
    mp_pose.PoseLandmark.NOSE,  # Nariz
    mp_pose.PoseLandmark.LEFT_EYE_INNER,  # Parte interna del ojo izquierdo
    mp_pose.PoseLandmark.LEFT_EYE,  # Ojo izquierdo
    mp_pose.PoseLandmark.LEFT_EYE_OUTER,  # Parte externa del ojo izquierdo
    mp_pose.PoseLandmark.RIGHT_EYE_INNER,  # Parte interna del ojo derecho
    mp_pose.PoseLandmark.RIGHT_EYE,  # Ojo derecho
    mp_pose.PoseLandmark.RIGHT_EYE_OUTER,  # Parte externa del ojo derecho
    mp_pose.PoseLandmark.RIGHT_EAR,  # Oreja derecha
    mp_pose.PoseLandmark.LEFT_EAR,  # Oreja izquierda
    mp_pose.PoseLandmark.RIGHT_PINKY,  # Meñique derecho
    mp_pose.PoseLandmark.LEFT_PINKY,  # Meñique izquierdo
    mp_pose.PoseLandmark.RIGHT_INDEX,  # Índice derecho
    mp_pose.PoseLandmark.LEFT_INDEX,  # Índice izquierdo
    mp_pose.PoseLandmark.RIGHT_THUMB,  # Pulgar derecho
    mp_pose.PoseLandmark.LEFT_THUMB,  # Pulgar izquierdo
    mp_pose.PoseLandmark.MOUTH_LEFT,  # Lado izquierdo de la boca
    mp_pose.PoseLandmark.MOUTH_RIGHT,  # Lado derecho de la boca
    mp_pose.PoseLandmark.RIGHT_HIP,  # Cadera derecha
    mp_pose.PoseLandmark.LEFT_HIP,  # Cadera izquierda
    mp_pose.PoseLandmark.LEFT_KNEE,  # Rodilla izquierda
    mp_pose.PoseLandmark.RIGHT_KNEE,  # Rodilla derecha
    mp_pose.PoseLandmark.RIGHT_HEEL,  # Talón derecho
    mp_pose.PoseLandmark.LEFT_HEEL,  # Talón izquierdo
    mp_pose.PoseLandmark.RIGHT_ANKLE,  # Tobillo derecho
    mp_pose.PoseLandmark.LEFT_ANKLE,  # Tobillo izquierdo
    mp_pose.PoseLandmark.RIGHT_FOOT_INDEX,  # Índice del pie derecho
    mp_pose.PoseLandmark.LEFT_FOOT_INDEX  # Índice del pie izquierdo
]

def hide_face_landmarks(landmarks):
    """Oculta los puntos específicos de la cara estableciendo su visibilidad en cero."""
    for i, landmark in enumerate(landmarks.landmark):
        if mp_pose.PoseLandmark(i) in EXCLUDE_FACE_LANDMARKS:
            landmark.visibility = 0  # Oculta el landmark de la cara
    return landmarks

def finger_closed(hand_landmarks, finger_tip, finger_pip):
    """Verifica si un dedo está cerrado comparando la posición de su punta con su articulación."""
    return hand_landmarks.landmark[finger_tip].y > hand_landmarks.landmark[finger_pip].y

def which_hand(result):
    """Determina cuál mano está siendo procesada."""
    return results_hands.multi_handedness[0].classification[0].label

# Inicializamos el nodo de ROS
rospy.init_node('Mediapipe', anonymous=True)
# Creamos un publicador para el tópico '/opcion'
pub = rospy.Publisher('/opcion', Int32, queue_size=10)

# Variable para almacenar la opción actual
opcion = 0

# Configuración para pose y manos
with mp_pose.Pose(
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5) as pose, \
     mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=2,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5) as hands:
    
    # Captura de video desde la cámara
    cap = cv2.VideoCapture(0)
    while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")  # Ignoramos frames vacíos
            continue

        # Convierte la imagen de BGR a RGB para procesarla
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # Procesamos la pose y las manos
        results_pose = pose.process(image)
        results_hands = hands.process(image)

        # Verificamos si se detectaron landmarks de pose antes de ocultar los de la cara
        hidden_landmarks = None
        if results_pose.pose_landmarks is not None:
            hidden_landmarks = hide_face_landmarks(results_pose.pose_landmarks)

        # Procesamos las manos si se detectaron
        if results_hands.multi_hand_landmarks is not None:
            for hand_landmarks in results_hands.multi_hand_landmarks:
                # Dibujamos los landmarks de las manos
                mp_drawing.draw_landmarks(
                    image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                
                # Si solo hay una mano detectada
                if len(results_hands.multi_hand_landmarks) == 1:

                    # Determinamos si los dedos están levantados o cerrados
                    index_up = not finger_closed(hand_landmarks, index_tip, index_pip)
                    middle_up = not finger_closed(hand_landmarks, middle_tip, middle_pip)
                    ring_up = not finger_closed(hand_landmarks, ring_tip, ring_pip)
                    pinky_up = not finger_closed(hand_landmarks, pinky_tip, pinky_pip)

                    if hidden_landmarks:
                        # Obtenemos landmarks específicos de la pose
                        left_shoulder = hidden_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER]
                        left_wrist = hidden_landmarks.landmark[mp_pose.PoseLandmark.LEFT_WRIST]
                        
                        # Selección de modo según la posición de los dedos
                        if index_up and not middle_up and not ring_up and not pinky_up and opcion == 0:
                            # Un dedo levantado
                            opcion = 1
                            pub.publish(1)
                            
                        if index_up and middle_up and not ring_up and not pinky_up and opcion == 0:
                            # Dos dedos levantados
                            opcion = 2
                            pub.publish(2)

                        if index_up and middle_up and ring_up and not pinky_up and opcion == 0:
                            # Tres dedos levantados
                            opcion = 3
                            pub.publish(3)

                        if index_up and not middle_up and not ring_up and pinky_up and opcion == 0:
                            # Rock and roll (índice y meñique levantados)
                            opcion = 4
                            pub.publish(4)
                                
                        if index_up and middle_up and ring_up and pinky_up:
                            # Mano completamente abierta
                            opcion = 0

        # Convertimos la imagen de vuelta a BGR para mostrarla
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        # Dibuja los puntos de referencia de la pose sin los de la cara
        if results_pose.pose_landmarks and hidden_landmarks:
            mp_drawing.draw_landmarks(
                image,
                hidden_landmarks,
                mp_pose.POSE_CONNECTIONS,
                landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())
            
            # Calculamos un rectángulo alrededor de los landmarks de la pose
            landmarks = results_pose.pose_landmarks.landmark

            h, w, _ = image.shape
            x_min = round(min([landmark.x * w for landmark in landmarks]))
            y_min = round(min([landmark.y * h for landmark in landmarks]))
            x_max = round(max([landmark.x * w for landmark in landmarks]))
            y_max = round(max([landmark.y * h for landmark in landmarks]))

            # Dibujamos el rectángulo en la imagen
            imagen2 = cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (255, 0, 255), 10)
            
        # Mostramos la imagen en la ventana
        cv2.imshow('MediaPipe Pose and Hands', cv2.flip(imagen2, 1))
        
        # Presiona la tecla "ESC" para salir
        if cv2.waitKey(5) & 0xFF == 27:
            break

    # Liberamos la cámara
    cap.release()
