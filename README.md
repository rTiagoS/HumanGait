## "Classificação de Marcha Anômala utilizando técnicas de Machine Learning e Sensores Embarcados."

### Publicações

Para mais detalhes, verifique nosso artigo: https://arxiv.org/abs/2110.06139

Caso nosso dataset seja utilizado, por favor, nos cite utilizando a seguinte referência: &nbsp;
```
@misc{sa2021classification,
      title={Classification of anomalous gait using Machine Learning techniques and embedded sensors}, 
      author={T. R. D. Sa and C. M. S. Figueiredo},
      year={2021},
      eprint={2110.06139},
      archivePrefix={arXiv},
      primaryClass={eess.SP}
}
```

Link para o dataset: https://drive.google.com/drive/folders/1B0GRPU912k93h7YRrBrzX0vYdbw2wq_Z?usp=sharing

##### Este repositório, em síntese, viabiliza o acesso aos três módulos que implementam o projeto proposto. Os módulos são:

**1. Módulo da Aplicação Embarcada**;
   * Modelagem do esquemático e layout do hardware embarcado;
   * Integração do sensor MPU-6050 ao microcontrolador ESP32 DevKit C;
   * Integração dos dados coletados para um servidor Broker MQTT (Internet das Coisas);
  
  Tecnologias utilizadas:
   * Linguagens de programação C e C++ (implementação do software embarcado);
   * KiCAD para modelagem da PCB;
  
**2. Módulo da Aplicação Web**;
   * Implementação de uma interface para inserção de dados;
   * Integração ao ESP32 DevKit C e servidor Broker MQTT para testes de conexão;
   * Rotulação automática dos dados coletados para alimentar o dataset;
  
  Tecnologias utilizadas: 
  - Servidor Linux (Ubuntu 20.04) hospedando Node-RED e um servidor Broker MQTT Eclipse Mosquitto.
  
**3. Módulo da Aplicação IA**;
   * Implementação de diferentes modelos baseados em Machine Learning com a biblioteca scikit learn;
   * Implementação de diferentes modelos baseados em arquiteturas de Deep Learning com as bibliotecas Keras e Tensorflow;
  
  Tecnologias utilizadas:
   * Servidor Linux com GPU para treinamento de modelos de Deep Learning e Machine Learning;
   * Linguagem de programação Python para implementação dos modelos IA e rotinas de rotulagem automática do dataset;
  
  
 Para cada módulo, há um diretório com o respectivo nome de referência. Neles há os arquivos de implementação e entre outros detalhes.
 

 
