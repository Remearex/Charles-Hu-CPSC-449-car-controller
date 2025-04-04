WIDTH, HEIGHT = 800, 600


# Wall 1 points:
w1_1 = (WIDTH/4-50, HEIGHT)
w1_2 = (WIDTH/4-50, 3*HEIGHT/4-50)
w1_3 = (WIDTH/4+50, 3*HEIGHT/4-150)
w1_4 = (WIDTH/2+100, 3*HEIGHT/4-150)
w1_5 = (3*WIDTH/4, HEIGHT/5+30)
w1_6 = (WIDTH/2, HEIGHT/5)
w1_7 = (WIDTH/4, HEIGHT/2)
w1_8 = (0, HEIGHT/2)

# Wall 2 points:
w2_1 = (WIDTH/4+50, HEIGHT)
w2_2 = (WIDTH/4+50, 3*HEIGHT/4)
w2_3 = (WIDTH/4+100, 3*HEIGHT/4-50)
w2_4 = (3*WIDTH/4, 3*HEIGHT/4-50)
w2_5 = (WIDTH, 40)
w2_6 = (WIDTH/2-30, 40)
w2_7 = (WIDTH/4, HEIGHT/2-100)
w2_8 = (0, HEIGHT/2-100)

training_track = [[0, 0, WIDTH, 0], [WIDTH, 0, WIDTH, HEIGHT], [WIDTH, HEIGHT, 0, HEIGHT], [0, HEIGHT, 0, 0],
                  # [20, 20, WIDTH-20, 20], [WIDTH-20, 20, WIDTH-20, HEIGHT-20], [WIDTH-20, HEIGHT-20, 20, HEIGHT-20], [20, HEIGHT-20, 20, 20],
                  [w1_1[0], w1_1[1], w1_2[0], w1_2[1]], [w1_2[0], w1_2[1], w1_3[0], w1_3[1]], [w1_3[0], w1_3[1], w1_4[0], w1_4[1]], [w1_4[0], w1_4[1], w1_5[0], w1_5[1]],
                  [w1_5[0], w1_5[1], w1_6[0], w1_6[1]], [w1_6[0], w1_6[1], w1_7[0], w1_7[1]], [w1_7[0], w1_7[1], w1_8[0], w1_8[1]],
                  
                  [w2_1[0], w2_1[1], w2_2[0], w2_2[1]], [w2_2[0], w2_2[1], w2_3[0], w2_3[1]], [w2_3[0], w2_3[1], w2_4[0], w2_4[1]], [w2_4[0], w2_4[1], w2_5[0], w2_5[1]],
                  [w2_5[0], w2_5[1], w2_6[0], w2_6[1]], [w2_6[0], w2_6[1], w2_7[0], w2_7[1]], [w2_7[0], w2_7[1], w2_8[0], w2_8[1]]]