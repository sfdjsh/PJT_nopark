#!/usr/bin/env python
# _*_ coding: utf-8 _*_

import boto3

ACCESS_KEY_ID = 'AKIA2LPAPXE5XYE327MZ' #s3 관련 권한을 가진 IAM계정 정보
ACCESS_SECRET_KEY = '4z27F0WUnIZi9d7H8zwyW3r//EssztCQqWoxf0tw'
BUCKET_NAME = 'gotothemars'             # 버킷 이름
OBJECT_PATH = 'car_picture/card.jpg'    # S3에 올릴 파일 경로
FILE_PATH = 'card.jpg'                  # 로컬 파일의 경로

"""
사전 작업
pip2 install boto3 awscli
aws configure
ACCESS_KEY_ID, ACCESS_SECRET_KEY, BUCKET_NAME 순서대로 입력 후 엔터
마지막 항목은 그냥 엔터
객체 URL: https://gotothemars.s3.ap-northeast-2.amazonaws.com/car_picture/{car_num 들어갈 곳}.jpg
"""

s3 = boto3.client('s3')
s3.upload_file(FILE_PATH, BUCKET_NAME, OBJECT_PATH)
s3.download_file(BUCKET_NAME, OBJECT_PATH, FILE_PATH)