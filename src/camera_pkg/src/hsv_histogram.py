import cv2
import numpy as np
import matplotlib.pyplot as plt

def plot_hsv_histogram(image_path):
    # Cargar la imagen en formato BGR
    img = cv2.imread(image_path)

    # Convertir la imagen a espacio de color HSV
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Calcular los histogramas para cada canal de color HSV
    h_hist = cv2.calcHist([hsv_img], [0], None, [256], [0, 256])
    s_hist = cv2.calcHist([hsv_img], [1], None, [256], [0, 256])
    v_hist = cv2.calcHist([hsv_img], [2], None, [256], [0, 256])

    # Crear la figura y los subgráficos
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 6))

    # Graficar el histograma de Hue (H)
    ax1.plot(h_hist, color='r')
    ax1.set_title('Hue Histogram')

    # Graficar el histograma de Saturation (S)
    ax2.plot(s_hist, color='g')
    ax2.set_title('Saturation Histogram')

    # Graficar el histograma de Value (V)
    ax3.plot(v_hist, color='b')
    ax3.set_title('Value Histogram')

    # Ajustar el diseño y mostrar la gráfica
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # Ruta de la imagen de entrada
    image_path = '/home/pedro/camera_infrastructure_ws/src/camera_pkg/src/tennis_ball.jpg'

    # Llamar a la función para graficar los histogramas HSV
    plot_hsv_histogram(image_path)