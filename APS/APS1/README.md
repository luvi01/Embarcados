# Resumo :

Este projeto tem como objetivo a criação de um programa para o embarcado ATMEL SAME70, capaz de tocar músicas a partir de um buzzer. Além disso o sistema deve trocar de música por meio de botões, ter funcionalidade pause/play, representação visual da faixa tocada e piscar os leds no ritmo da mesma.

Vídeo do projeto: https://drive.google.com/open?id=1eaSwhrywcAwf1M0y3g0GuGRmNPeQOz6K


# Materiais :

Microcontrolador Atmel SAME70 Xplained
Periférico OLED1 Xplained
Buzzer 5V
Jumpers macho-fêmea

O Button do periférico OLED1 seleciona a música 1
O Button do periférico OLED1 seleciona a música 2
O Button do periférico OLED1 seleciona a música 3
O Button SW0 da placa é responsável por realizar a funcionalidade de pause/play


# Programa :

O arquivo musicas.h contém a frequência de todas as notas musicais requeridas pelas músicas, existem nele também as sequências de notas musicais junto com as suas durações para todas as melodias do programa. Estas são combinadas no main.c em struts, para depois serem alimentadas na função “play_music(note_s melody[], double songspeed, int n)”, que por sua vez faz diversas chamadas a função “void tone(int frequency, double duration)”, que é responsável por gerar uma onda quadrada com a frequência e duração passadas pela música, para o buzzer. Assim criando a música.

<img src="diagrama.png" alt="esquema" width=75% height=75% />

