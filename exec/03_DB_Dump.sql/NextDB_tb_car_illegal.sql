-- MySQL dump 10.13  Distrib 8.0.29, for Win64 (x86_64)
--
-- Host: j7c109.p.ssafy.io    Database: NextDB
-- ------------------------------------------------------
-- Server version	8.0.30-0ubuntu0.20.04.2

/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!50503 SET NAMES utf8 */;
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0 */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*!40111 SET @OLD_SQL_NOTES=@@SQL_NOTES, SQL_NOTES=0 */;

--
-- Table structure for table `tb_car_illegal`
--

DROP TABLE IF EXISTS `tb_car_illegal`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `tb_car_illegal` (
  `car_num` varchar(100) NOT NULL,
  `time` datetime DEFAULT NULL,
  `npc_x` double DEFAULT NULL,
  `npc_y` double DEFAULT NULL,
  `prior_cnt` int DEFAULT NULL,
  `image_data` varchar(1000) NOT NULL,
  PRIMARY KEY (`car_num`)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `tb_car_illegal`
--

LOCK TABLES `tb_car_illegal` WRITE;
/*!40000 ALTER TABLE `tb_car_illegal` DISABLE KEYS */;
INSERT INTO `tb_car_illegal` VALUES ('2016_Hyundai_Santafe','2022-10-07 00:22:11',147.7515869140625,1424.3165283203125,7,'https://gotothemars.s3.ap-northeast-2.amazonaws.com/car_picture/2016_Hyundai_Santafe.jpg'),('2019_Hyundai_Nexo','2022-10-07 00:22:05',142.94174194335938,1349.603759765625,7,'https://gotothemars.s3.ap-northeast-2.amazonaws.com/car_picture/2019_Hyundai_Nexo.jpg'),('2020_Kia_Stinger','2022-10-07 00:04:44',147.35491943359375,1537.41796875,6,'https://gotothemars.s3.ap-northeast-2.amazonaws.com/car_picture/2020_Kia_Stinger.jpg'),('2021_Volkswagen_Golf_GTI','2022-10-07 00:21:59',116.85798645019531,1274.744140625,7,'https://gotothemars.s3.ap-northeast-2.amazonaws.com/car_picture/2021_Volkswagen_Golf_GTI.jpg');
/*!40000 ALTER TABLE `tb_car_illegal` ENABLE KEYS */;
UNLOCK TABLES;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2022-10-07  2:30:12
