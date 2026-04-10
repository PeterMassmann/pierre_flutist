import json
from datetime import datetime

negra = 60
notas = []
tiempos = []

with open("scores/Etude Mecanique/etude_mechanique.txt", "r") as file:
    for line in file.readlines():
        line = line.replace("\n", "")
        if line[0] == "#":
            if "Negra = " in line:
                negra = int(line.replace(" ", "").split("=")[-1])
        else:
            info = line.split(" ")
            for i in range(len(info) // 2):
                notas.append(info[i*2])
                if "/" in info[i*2+1]:
                    divs = info[i*2+1].split("/")
                    t = int(divs[0]) / int(divs[1])
                else:
                    t = int(info[i*2+1])
                tiempos.append(t*60/negra)

dedos = []
t = 0

for i in range(len(notas)):
    nota = notas[i].replace("Db", "C#").replace("Eb", "D#").replace("Gb", "F#").replace("Ab", "G#").replace("Bb", "A#")
    if "S" not in nota:
        dedos.append([t, nota])
    t += tiempos[i]

with open("scores/Etude Mecanique/etude_mechanique.json") as json_file:
    partitura = json.load(json_file)

partitura['route_fingers']['notes'] = dedos
partitura['route_fingers']['total_t'] = t
partitura['route_flow']['total_t'] = t
partitura['route_offset']['total_t'] = t
partitura['route_r']['total_t'] = t
partitura['route_theta']['total_t'] = t
partitura['timestamp'] = datetime.now().strftime("%d/%m/%Y %H:%M:%S")

with open("scores/Etude Mecanique/etude_mechanique.json", 'w') as json_file:
    json.dump(partitura, json_file, indent=4, sort_keys=True)

print(partitura)