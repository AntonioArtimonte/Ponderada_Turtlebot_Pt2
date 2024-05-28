# Teleoperação do Robô com Feed de Câmera e Controle de Segurança

Este projeto é uma interface de teleoperação para um robô utilizando ROS2 e Pygame. Inclui funcionalidades para controlar os movimentos do robô, exibir o feed da câmera e implementar recursos de segurança usando dados LiDAR.

## Requisitos

- Python 3.7+
- ROS2
- Pygame
- OpenCV
- Pillow
- NumPy

## Instalação

### Ubuntu e ROS2

Primeiramente, deve-se rodar o código apenas em máquinas Ubuntu 22.04, se a sua é algum outro `Distro`, instale o Ubuntu 22.04 através do [link](https://releases.ubuntu.com/jammy/)

Após se certificar de que possui o Ubuntu em sua máquina, primeiramente rode no terminal do mesmo o seguinte comando afim de checar se o ROS2 está ou não instalado:

```bash
ros2
```

Se o mesmo retornar comando inexistente, siga o [guia de instalação do ROS2](https://docs.ros.org/en/foxy/Installation.html).

### Python3

Após se certificar de ter o ROS2 e Ubuntu instalados, execute em seu terminal o seguinte comando afim de checar se o python está instalado:

```bash
which python3
```

Se o mesmo retornar python not found, digite no terminal o seguinte comando para instalar o python:

```bash
sudo apt update
sudo apt install python3
```

#### Dependências python3

Afim de rodar o código principal, será necessário instalar alguns módulos do python, para fazer o mesmo digite o seguinte no terminal:

```bash
pip install pygame opencv-python pillow numpy
```

## Inicialização do código

Afim de rodar o código, é necessário rodar tanto o scrip do mesmo em sua maquina local, tanto quanto o script de receber e enviar informações dentro do robô. Para fazer o mesmo deve-se seguir os passos abaixo:

Obs: **!!Deve-se estar na rede Inteli.Iot, caso contrário não ira funcionar!!**

### Inicialização do robô

1. Conectar via SSH com o robô:

```bash
ssh grupo4@10.128.0.9
```

2. Entrar no diretório do workspace principal:

```bash
cd main_ws
```

3. Fontar o workspace

```bash
source install/local_setup.bash
```

4. Entrar no diretório de launch files e rodar o launch file específico

```bash
cd launch
ros2 launch launch.py
```

Após isto, cheque se o terminal ira mostrar várias vezes a seguinte mensagem: `Publishing image as base64 string`. Se estiver, tudo deu certo, caso contrário desligue o robô e repita o processo descrito acima.

### Inicialização do código do pygame

Após se certificar de ter ligado o robô, e o código do mesmo estar funcionando, execute o seguinte código em seu terminal de sua máquina local:

1. Clone o repositório:

```bash
git clone https://github.com/AntonioArtimonte/Ponderada_Turtlebot_Pt2
```

2. Entre no diretório principal:

```bash
cd PonderadaS6
```

3. Instale as depêndencias do python (se não chegou a instalar antes):

```bash
pip install pygame opencv-python pillow numpy
```

4. Execute o código principal

```bash
python3 main.py
```

Após isto, está pronto para utilização.

## Visão Geral do Código

### Inicialização do Pygame

A biblioteca Pygame é inicializada e o display é configurado com uma resolução de 1680x720. As propriedades dos botões, como cor, fonte e posição, são definidas.

### Desenho dos Botões

A função `draw_buttons` é usada para renderizar botões na tela do Pygame. Cada botão é desenhado como um retângulo com um rótulo.

### Controlador do Robô

A classe `RobotController` herda de `Node` e inclui métodos para controlar o movimento do robô. Ela se inscreve nos dados LiDAR para implementar recursos de segurança e possui métodos para aumentar/diminuir velocidades lineares e angulares.

### Ouvinte

A classe `Listener` herda de `Node` e se inscreve nos dados do feed da câmera. Ela decodifica as imagens recebidas e calcula a latência, que é armazenada em uma fila para atualizações da interface.

### Inicialização dos Nós ROS

A função `init_ros_nodes` inicializa os nós ROS para o controlador do robô e o ouvinte da câmera. Os nós são executados em uma thread separada usando a função `spin_nodes`.

### Loop Principal

A função `main` contém o loop principal para a aplicação Pygame. Ela lida com eventos como pressionamentos de teclas e cliques do mouse para controlar o robô. Também atualiza o feed da câmera, exibe a latência e as velocidades do robô na tela.

### Latência Média Móvel

Uma lista `latencies` é usada para armazenar os valores de latência recentes, e a função `moving_average` calcula a média móvel das latências. Isso é usado para exibir a latência média em milissegundos.

## Como Usar

- **W**: Aumentar velocidade linear (mover para frente)
- **S**: Diminuir velocidade linear (mover para trás)
- **A**: Aumentar velocidade angular (virar à esquerda)
- **D**: Diminuir velocidade angular (virar à direita)
- **SPACE**: Parar o robô
- **Q**: Sair da aplicação
- **B**: Parada de emergência

Clicar nos botões na tela também controlará os movimentos do robô.

## Recursos de Segurança

O robô utiliza dados LiDAR para garantir a segurança. Se um obstáculo for detectado dentro de uma distância de segurança especificada, o robô parará automaticamente.

## Feed da Câmera

O feed da câmera é exibido na tela do Pygame. A latência do feed é calculada e exibida como uma média móvel em milissegundos.

## Vídeo de funcionamento

Também foi disponível um vídeo para exemplificar o funcionamento do robô. O mesmo pode ser acessado [aqui]()

## Observações

Os códigos que estão implementados no robô, estarão disponíveis dentro da pasta robot neste GitHub.

### Camera.py

O código camera.py, apenas pega em 60FPS (Frames Por Segundo) as imagens da câmera, codifica em uma string no formato Base64 e manda via tópico ROS.

### Emergency.py

O código emergency.py abre um tópico do ROS, além de iniciar em um subprocesso o bringup padrão do turtlebot3. E quando o mesmo escuta algo neste tópico, mais especificamente `emergency_stop`, ele mata o processo do bringup, não permitindo qualquer comunicação com o robô.

Código principal criado por `Antonio Artimonte Vaz Guimarães`