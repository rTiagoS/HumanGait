# "Marcha anômala: detecção e classificação a partir de um dispositivo wearable e técnicas de Machine Learning"

## Este repositório, em síntese, viabiliza o acesso aos três módulos que implementam o projeto proposto. Os módulos são:

1. Módulo da Aplicação Embarcada;
  1.1. Modelagem do esquemático e layout do hardware embarcado;
  1.2. Integração do sensor MPU-6050 ao microcontrolador ESP32 DevKit C;
  1.3. Integração dos dados coletados para um servidor Broker MQTT (Internet das Coisas);
  
  Tecnologias utilizadas: 
  - Linguagens de programação C e C++ (implementação do software embarcado),
  - KiCAD para modelagem da PCB;
  
2. Módulo da Aplicação Web;
  2.1 Implementação de uma interface para inserção de dados;
  2.2 Integração ao ESP32 DevKit C e servidor Broker MQTT para testes de conexão;
  2.3 Rotulação automática dos dados coletados para alimentar o dataset;
  
  Tecnologias utilizadas: 
  - Servidor Linux (Ubuntu 20.04) hospedando Node-RED e um servidor Broker MQTT Eclipse Mosquitto.
  
3. Módulo da Aplicação IA;
  3.1. Implementação de diferentes modelos baseados em Machine Learning com a biblioteca scikit learn;
  3.2. Implementação de diferentes modelos baseados em arquiteturas de Deep Learning com as bibliotecas Keras e Tensorflow;
  
  Tecnologias utilizadas:
  - Servidor Linux com GPU para treinamento de modelos de Deep Learning e Machine Learning;
  - Linguagem de programação Python para implementação dos modelos IA e rotinas de rotulagem automática do dataset;
  
  
 Para cada módulo, há um diretório com o respectivo nome de referência. Neles há os arquivos de implementação e entre outros detalhes.
 

 