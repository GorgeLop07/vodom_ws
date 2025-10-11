# Filtro de topicos y Tiempos en un rosbag (:O)
Este codigo fue basado en: https://github.com/armando-genis/navpilot-framework/blob/main/workspace/navpilot_ws/src/mcap_reader/filter_bag.py

## Que es lo que hace? 
- Este toma un rosbag de tipo .db3 o .mcap, y filtra los topicos indicados por el usuario 
- Recorta un inicio y final de los datos en caso de que sea necesario (Tomando en cuenta los timestamps de ROS ) 

## Como ejecutarlo:
### Paso 1 - Clonar el Repo:
```bash
git clone https://github.com/GorgeLop07/vodom_ws.git
```

### Paso 2 - Metete al directorio
```bash
cd vo_ws/src/rosbag_reader
```

### Paso 3 - Ejecuta el archivo
Para ejecutarlo tienes que tomar en cuenta que es un ejecutable PARAMETRIZABLE, osea puedes modificar los siguientes parametros: 

- **input_path:** Ruta al bag de entrada **(/PATH/LOL)**
- **output_path:** Ruta de salida **(/PATH/LOL)**
- **topics_to_keep:** Lista de tópicos a mantener (None = todos) **(--topic-file topic.txt)**
- **start_offset_s:** Segundos a omitir desde el inicio (default: 0)**(--start 100)**
- **end_offset_s:** Segundo donde terminar (desde el inicio del bag)**(--end 100)**
- **duration_s:** Duración en segundos a grabar (alternativa a end_offset_s) **(--duration 100)**
- **output_format:** "db3" o "mcap" **(--formar mcap)**

**Ejemplo de Ejecucion (Recortar TIempo):**
```bash
python3 trim_and_filter_rosbag.py \
/path/absoluto/a/tu/rosbag \
/path/absoluto/donde/guardarlo \
    --start 270 --duration 550 \
    --format mcap
```
.
**Ejemplo de Ejecucion (Quitar Topicos con topic.txt):**
``` bash
python3 db3FIlterToMcap.py \
/path/absoluto/a/tu/rosbag \
/path/absoluto/donde/guardarlo \
    --topics-file topic.txt \
    --format mcap
```
**SE PUEDEN COMBINAR PARAMETROS DE RECORTE Y FILTRADO** 

**Mas ejemplos dentro del codigo**




