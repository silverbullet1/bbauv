if __name__ == "__main__":
    rospy.init_node("ac")
    pub = rospy.Publisher("/acoustic/angFromPing", acoustic)    
    print("X_Pos: " + str(abs_pos['x']) + "\t\t" + "Y_Pos: " +  str(abs_pos['y']) + "\n")
    print heading['yaw']
    print("Depth: " + str(depth) + "\n")
    (conn, addr) = s.accept()
    data = conn.recv(BUFFER_SIZE)
    if('NaN' in data or 'Inf' in data):
        print "NaNster is here"
    else:
        print "Correct"
    ls = []
    listen_ping(ls,conn,data)
    music_algo(ls)
    print "Relative is: ", DOA, " degrees"
    #Ensure pinger always on right hand side
    sendMovement(h=(heading['yaw'] + (DOA - 20))%360)
    stopRobot()
    conn.close()
    (conn, addr) = s.accept()
    data = conn.recv(BUFFER_SIZE)
    listen_ping(ls)
    music_algo(ls)
    angFromPing.append(DOA)
    ls = []
    rospy.loginfo("Got First Point")
    if len(angFromPing) is not 2:
        stopRobot()
        rospy.loginfo("Going straight")
        sendMovement(f = forward, d=depth)
        stopRobot()
        ls = []
        conn.close()
        rospy.loginfo("Connection closed")
        for i in range(1000):
            pass
        (conn, addr) = s.accept()
        data = conn.recv(BUFFER_SIZE)
        listen_ping(ls, conn,data)
        music_algo(ls)
        angFromPing.append(DOA)
    a = neotriangulate(angFromPing[0], angFromPing[1], forward)
    print("Location of Pinger: Distance of " + str(a[1])+"\n")
    print a[0]
    sendMovement(h=(a[0] + heading['yaw'])%360)
    sendMovement(f=a[1])
    rospy.spin()
    conn.close()

