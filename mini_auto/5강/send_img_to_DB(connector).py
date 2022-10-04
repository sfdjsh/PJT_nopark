#!/usr/bin/env python
# _*_ coding: utf-8 _*_

import mysql.connector
from mysql.connector import Error

connection = mysql.connector.connect(
    host = 'j7c109.p.ssafy.io',
    database = 'NextDB',
    user = 'gotothemars',
    password = 'c109_mars'
)
cursor = connection.cursor()

def convertToBinaryData(filename):
    with open(filename, 'rb') as file:
        binaryData = file.read()
    return binaryData

def insertBLOB(name, photo):
    global connection, cursor
    print("inserting BLOB into images table")
    try:
        sql_insert_blob_query = "insert into tb_car_illegal (car_num, image_data) values (%s, %s)"
        picture = convertToBinaryData(photo)

        insert_blob_tuple = (name, picture)
        result = cursor.execute(sql_insert_blob_query, insert_blob_tuple)
        connection.commit()
        print("nyam", result)

    except Error as error:
        print(error)

    finally:
        if (connection.is_connected()):
            cursor.close()
            connection.close()
            print("MySQL closed")

def writeFile(data, filename):
    with open(filename, 'wb') as file:
        file.write(data)

def readBLOB(photo):
    global connection, cursor
    print("Reading BLOB data from images table")
    try:
        sql_fetch_blob_query = """
            SELECT * from tb_car_illegal where car_num = %s
        """

        cursor.execute(sql_fetch_blob_query, ("test1"))
        record = cursor.fetchall()
        for row in record:
            print(row)
            writeFile(photo)

    except Error as error:
        print(error)

    finally:
        if (connection.is_connected()):
            cursor.close()
            connection.close()
            print("MySQL closed")

if __name__ == "__main__":
    cmd = raw_input("뭘 할까요? ")
    if cmd == "in":
        insertBLOB("test1", "card.jpg")
    elif cmd == "out":
        readBLOB("card.jpg")