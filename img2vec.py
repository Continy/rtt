import cv2
import numpy as np


def img2vec(filepath, num):
    img = cv2.imread(filepath, cv2.IMREAD_GRAYSCALE)

    thresh, new_img = cv2.threshold(img, 150, 255, cv2.THRESH_BINARY)
    k = np.ones((4, 4), np.uint8)  #定义核
    img = cv2.dilate(new_img, k)
    img = cv2.erode(new_img, k)  # 腐蚀
    # cv2.circle(img, [1000, 0], 4, color=(0, 255, 255))
    # cv2.circle(img, [1000, 562], 4, color=(0, 255, 255))
    img = cv2.resize(img, (200, 224))
    vec = []
    for i in range(0, img.shape[0]):
        for j in range(0, img.shape[1]):
            if img[i][j] == 0:
                vec.append([i, j, 0.5])
        if i % 100 == 0:
            print(i)
    np.save("mid/" + str(num) + ".npy", vec)


if __name__ == '__main__':
    filepath = "img/4.png"
    namenum = 4
    img2vec(filepath, namenum)
    img = np.load("mid/" + str(namenum) + ".npy")

    print(img)
