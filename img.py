import matplotlib.pyplot as plt
import cv2

img_path = "image.png"  # <-- cambia esto
img = cv2.imread(img_path)

if img is None:
    raise FileNotFoundError(f"No pude leer {img_path}")

# Convertir BGR → RGB
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

fig, ax = plt.subplots()
ax.imshow(img)
ax.set_title("Click sobre el cubo")

def onclick(event):
    if event.xdata is not None and event.ydata is not None:
        x = int(event.xdata)
        y = int(event.ydata)
        r, g, b = img[y, x]
        print(f"CLICK -> x={x}, y={y} | RGB=({r},{g},{b})")

cid = fig.canvas.mpl_connect('button_press_event', onclick)

plt.show()
