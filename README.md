# Controllo Cinematico del Robot Kinova Gen3

[![MATLAB](https://img.shields.io/badge/MATLAB-R2023b-orange?logo=mathworks)](https://www.mathworks.com/)
[![Robotics](https://img.shields.io/badge/Robotics-Kinova%20Gen3-blue)](https://www.kinovarobotics.com/)
[![Status](https://img.shields.io/badge/Status-Completed-success)]()

Un progetto MATLAB per l'implementazione e il testing di un **controllore di cinematica inversa** per il braccio robotico Kinova Gen3 (7 DoF). Il progetto esplora diverse strategie di controllo per il tracking di traiettorie, dalla gestione della posizione fino all'ottimizzazione della ridondanza.

---

## Panoramica

Questo progetto implementa in MATLAB un controllore cinematico per il robot **Kinova Gen3**. L'obiettivo è far sì che l'end-effector del robot segua una traiettoria complessa, composta da segmenti rettilinei e una semicirconferenza, rispettando diversi vincoli di orientamento e ottimizzazione.

**Traiettoria Desiderata:**
<center>
<img src="https://github.com/user-attachments/assets/e055772a-ed1b-436a-9ab4-45c34f6d1ae1" width="300" alt="Diagramma della Traiettoria">
</center>

---

## Obiettivi del Progetto

Il progetto è suddiviso in quattro casi di studio progressivi:

| Caso | Obiettivo | 
|:----:|-----------|
| **1** | Controllo della sola **posizione** dell'end-effector. | 
| **2** | Controllo di posizione con **orientamento costante**. | 
| **3** | Controllo di posizione con **orientamento variabile** (asse Z punta al centro della traiettoria). | 
| **4** | **Ottimizzazione** con ridondanza per rispettare i limiti di giunto. | 

---

## Tecnologie e Strumenti

- **Linguaggio:** MATLAB
- **Robot:** Kinova Gen3 (7 DoF) in simulazione, Jaco (7 DoF in laboratorio)
- **Tecniche:** Cinematica Inversa, Quaternioni, Pseudoinversa Smorzata, Ottimizzazione della Ridondanza
- **Strumenti:** MATLAB Robotics System Toolbox

---

## Risultati e Casi di Studio

### Caso 1: Controllo della Posizione
Il robot segue la traiettoria senza vincoli sull'orientamento.
![Risultato Caso 1](https://github.com/user-attachments/assets/5eb3a7b8-4f60-4795-9b6a-2bac86539014)

### Caso 2: Posizione + Orientamento Costante
Mantiene un orientamento fisso di 45° lungo l'intera traiettoria, utilizzando **quaternioni** per il calcolo della rotazione.
![Risultato Caso 2](https://github.com/user-attachments/assets/b3bee8dd-4172-412b-bbf9-bdacf6ce3bbc)

### Caso 3: Posizione + Orientamento Variabile
Nella fase semicircolare, l'asse Z dell'end-effector è sempre rivolto verso il centro della traiettoria (implementato con `Rot2AxisAngle`).
![Risultato Caso 3](https://github.com/user-attachments/assets/a948569a-8134-4a69-a744-79d96105df7e)

### Caso 4: Ottimizzazione con Ridondanza
Utilizza la **pseudoinversa smorzata** per ottimizzare il movimento, rispettare i limiti fisici dei giunti e tornare alla configurazione iniziale.

---

## Come Usare il Codice

1.  **Prerequisiti:**
    - MATLAB installato (consigliata R2023b o superiore).
    - Robotics System Toolbox sia installato.

2.  **Clona il repository:**
    ```bash
    git clone https://github.com/noemilatorre/Progetto-Kinova-Gen3.git
    cd Progetto-Kinova-Gen3
    ```

3.  **Esegui lo script:**
    - Aprire MATLAB e navigare nella cartella del progetto.
    - Eseguire lo script in cui è presente un flag per i 4 casi
    

---

## Contatti

**Noemi La Torre**

- Email: latorre.noemi17@gmail.com
- LinkedIn: [linkedin.com/noemi-la-torre](https://www.linkedin.com/in/noemi-la-torre)
- GitHub: [github.com/noemilatorre](https://github.com/noemilatorre)
- Portfolio: [noemilatorre.github.io](https://noemilatorre.github.io)

---
*Questo progetto è stato sviluppato come parte del corso di Sistemi Robotici e Robotica Applicata presso l'Università degli Studi di Cassino e del Lazio Meridionale.*
