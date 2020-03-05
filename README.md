# Info-2019_Petit_Robot



* Code_tableau/ : Je ne sais plus d'où ça vient, c'est sans doutes le programme sur lequel je me suis basé pour le PID.
* Petit_Robot/ : Contient l'entièreté du code pour le Petit Robot 2019 ("Shlagito"), excepté deux libraries pour les capteurs lasers et ultrason (qui se trouvent dans libraries/) et la library de base Wire qui est intégrée dans la toolchain Arduino.
* Petit_Robot/Petit_Robot.ino : Programme principale pour la coupe 2019
* Petit_Robot/Petit_Robot-TNEV.ino : Programme alternatif pour réaliser le slalom de TNEV et leur montrer que c'est pas compliqué (et se moquer. un peu.)

Note: La toolchain Arduino n'accepte qu'un seul fichier .ino, il faut donc renommer (.bak) l'un des deux fichiers pour compiler avec l'autre.
