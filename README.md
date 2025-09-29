# Progetto-Kinova-Gen3

## Descrizione  
Questo progetto riguarda il controllo del robot **Kinova Gen3**, un braccio robotico a **7 gradi di libertà**, utilizzando **MATLAB**.  
L'obiettivo è implementare e testare un **controllore di cinematica inversa** con obiettivi differenti.  

---

## Obiettivi del progetto  
- **Caso 1:** Controllo solo posizione dell'end-effector senza considerare l'orientamento.  
- **Caso 2:** Controllo della posizione con orientamento costante.  
- **Caso 3:** Controllo della posizione con orientamento variabile (asse Z dell'ee sempre rivolto al centro della traiettoria).  
- **Caso 4:** Ottimizzazione con ridondanza per rispettare i limiti di giunto e tornare alla configurazione iniziale.  

---
## Traiettoria desiderata
<center>
<img src="https://github.com/user-attachments/assets/e055772a-ed1b-436a-9ab4-45c34f6d1ae1" width=300>
</center>


---

## Casi di Studio e Risultati  

### Caso 1: Controllo della posizione  
Il robot segue una traiettoria composta da:  
1️. Un primo segmento rettilineo  
2️. Una semicirconferenza verticale  
3️. Un secondo segmento per tornare alla posizione iniziale  

![image](https://github.com/user-attachments/assets/5eb3a7b8-4f60-4795-9b6a-2bac86539014)
  

---

### Caso 2: Controllo della posizione e orientamento costante  
- L'orientamento iniziale di 45° viene mantenuto lungo tutta la traiettoria.  
- La rotazione è implementata usando **quaternioni**.  

![image](https://github.com/user-attachments/assets/b3bee8dd-4172-412b-bbf9-bdacf6ce3bbc)
 

---

### Caso 3: Controllo della posizione e orientamento variabile  
- Durante il tratto semicircolare, l'asse **Z dell'end-effector** punta sempre verso il centro della traiettoria.  
- La rotazione viene calcolata usando la funzione **Rot2AxisAngle**.  

![image](https://github.com/user-attachments/assets/a948569a-8134-4a69-a744-79d96105df7e)
  

---

### Caso 4: Ottimizzazione con ridondanza  
- L'algoritmo ottimizza il movimento per rispettare i **limiti di giunto** e tornare alla configurazione iniziale.  
- L'ottimizzazione viene gestita con la **pseudoinversa smorzata**.  


---

