% Clean workspace
clear
close all
clc
%% FLAG
FLAG.robot = 'gen3'; % 'jaco'
opzione = 'inversa'; %inversa - trasposta

%% Flag 4 casi
 % caso 1 = Controllo solo posizione
 % caso 2 = Posizione e orientamento costante
 % caso 3 = Semicirconferenza con z_ee che punta verso il centro
 % caso 4 = Uso ridondanza per ritorno a q_home
FLAG.caso = 1;

%% Parametri gen3
fprintf('\n gen3 7 links \n')

a     = [0 0 0 0 0 0 0]';
alpha = [pi/2 pi/2 pi/2 pi/2 pi/2 pi/2 0]';
d     = [0.2848 -0.0054 -0.4208 0.0128 -0.3143 0 0.1674]';
theta = [77  -17 0  43 -94 77 71]'/180*pi; % approximated home configuration

DH = [a alpha d theta];

robot = loadrobot('kinovaGen3','DataFormat','row','Gravity',[0 0 -9.81]);

%% Selezione con flag per inversa o trasposta
if strcmp(opzione, 'inversa')
    K = diag([20*[1 1 1], 10*[1 1 1]]);
    fprintf('\n opzione: inversa Jacobiano')
else
    K = diag([30*[1 1 1], 50*[1 1 1]]);
    fprintf('\n opzione: trasposta Jacobiano')
end
fprintf('\n')

%% Parametri simulazione
tf = 29;              % fine simulazione
T = 1e-3;             % passo temporale
n = size(DH, 1);      % numero di giunti
t = 0:T:tf;
N = length(t);

%% Definizione variabili 
q   = zeros(n, N);
q(:,1) = DH(:,4);     % inizializzazione posizione giunti
dq = zeros(n, N);     % velocità angolare dei giunti
dq_d = zeros(n,N);

p_d = zeros(3, N);    % posizione desiderata
p = zeros(3, N);      % posizione corrente
quat = zeros(4, N);   % quaternione dell'end-effector
quat_d = zeros(4, N); % quaternione desiderato

error_pos = zeros(3, N);    % errore di posizione
error_quat = zeros(3, N);   % errore di quaternione
error = zeros(6, N);        % errore 
cond_J = zeros(N, 1);       % condizionamento Jacobiano

T0 = DirectKinematics(DH);

%% Parametri traiettoria desiderata
p0 = T0(1:3,4,n);           % posizione iniziale e finale (home)
p_target = [0; -0.3; 0.2];

traj_duration_linear = 5;   % durata tratto lineare
traj_duration_circle = 16;  % durata traiettoria semicircolare
traj_duration_home = 7;     % durata del ritorno a home
pause_duration = 0.5;

ro = 0.1;                    % raggio della circonferenza 10 cm
c = p_target + ro*[1; 0; 0]; % centro del cerchio

R = eye(3);
R_i = T0(1:3,1:3,n);         % mat di rotazione iniziale dell'end-effector 

x_des = zeros(3,1);
y_des = zeros(3,1);
z_des = zeros(3,1);  

z_in = T0(1:3,3,n);          % asse di rotazione in terna e.e.
q_home = DH(:, 4);
dq_c=[.27 .25 .12 .11 .35 .04 .25]';

segment_length = norm(p_target - p0);      % den traiettoria lineale
segment_length_home = norm(p0 - p_target); % lunghezza segmento verso home
temp = 0; %variabile temporanea 

%% Limiti di saturazione sulla velocità
joint_lower_limit = [-0.81; -0.81; -0.81; -0.81; -1.11; -1.11; -1.11];
joint_upper_limit = [0.81; 0.81; 0.81; 0.81; 1.11; 1.11; 1.11];


for i = 1:N

switch FLAG.caso
    %% CASO 1 SOLO POSIZIONE 
    case 1
       if i == 1
           disp('Caso 1: Controllo solo posizione');
       end    

       % Tratto lineare iniziale
       if t(i) <= traj_duration_linear
           s_pos = trapezoidal(0, segment_length, 0.087, traj_duration_linear, t(i));
           p_d(:, i) = p0 + (s_pos / segment_length) * (p_target - p0);

       % Pausa 500 ms dopo il tratto lineare
       elseif t(i) <= traj_duration_linear + pause_duration
           p_d(:, i) = p_d(:, i - 1); 

       % Semicirconferenza
       elseif t(i) <= traj_duration_linear + pause_duration + traj_duration_circle
           t_circle = t(i) - (traj_duration_linear + pause_duration);
           s_pos = trapezoidal(0, pi * ro, 0.03, traj_duration_circle - pause_duration, t_circle);
           p_d(:, i) = c + R * [-ro * cos(s_pos / ro); 0; ro * sin(s_pos / ro)];

       % Pausa dopo la semicirconferenza
       elseif t(i) <= traj_duration_linear + pause_duration + traj_duration_circle + pause_duration
           p_d(:, i) = p_d(:, i - 1); 

       % Ritorno alla posizione home (p0)
       elseif t(i) <= traj_duration_linear + pause_duration + traj_duration_circle + pause_duration + traj_duration_home
           t_home = t(i) - (traj_duration_linear + pause_duration + traj_duration_circle + pause_duration);
       % trovo il numero di passi temporali necessari per completare i due tratti,
       % li sommo e poi con round() estrae indice colonna p_d (ultimo elemento)
           p_final_circle = p_d(:, round((traj_duration_linear + pause_duration)/T) + round(traj_duration_circle/T));
           segment_length_home = norm(p0 - p_final_circle);

           s_pos_home = trapezoidal(0, segment_length_home, 0.1055, traj_duration_home - pause_duration, t_home);
           p_d(:, i) = p_final_circle + (s_pos_home / segment_length_home) * (p0 - p_final_circle);

       else      
           p_d(:, i) = p0;
       end

%% CASO 2 POSIZIONE E ORIENTAMENTO COSTANTE
case 2

   if i == 1
       disp('Caso 2: Controllo posizione e orientamento costante');
   end   

   % Tratto lineare iniziale
   if t(i) <= traj_duration_linear
       
       s_pos = trapezoidal(0, segment_length, 0.0867, traj_duration_linear, t(i)); 
       p_d(:, i) = p0 + (s_pos / segment_length) * (p_target - p0);

       % Rotazione di 45° su x 
       angolo = pi/4;
       s_or = trapezoidal(0, angolo, 0.197, traj_duration_linear, t(i));
       R_d_current = R_i * Rot_axisangle([1 0 0]', s_or);  % Ruoto rispetto all'asse x
      
       quat_d(:, i) = Rot2Quat(R_d_current); 

   % Pausa dopo il tratto lineare
   elseif t(i) <= traj_duration_linear + pause_duration
       p_d(:, i) = p_d(:, i - 1); 
       quat_d(:, i) = Rot2Quat(R_d_current); 

   % Semicirconferenza
   elseif t(i) <= traj_duration_linear + pause_duration + traj_duration_circle
       t_circle = t(i) - (traj_duration_linear + pause_duration);
       s_pos = trapezoidal(0, pi * ro, 0.024, traj_duration_circle - pause_duration, t_circle);
       p_d(:, i) = c + R * [-ro * cos(s_pos / ro); 0; ro * sin(s_pos / ro)];

       % Orientamento costante
       quat_d(:, i) = Rot2Quat(R_d_current); 

   % Pausa dopo la semicirconferenza
   elseif t(i) <= traj_duration_linear + pause_duration + traj_duration_circle + pause_duration
       p_d(:, i) = p_d(:, i - 1); 
       quat_d(:, i) = Rot2Quat(R_d_current); 

   % Ritorno alla posizione home
   elseif t(i) <= traj_duration_linear + pause_duration + traj_duration_circle + pause_duration + traj_duration_home
       t_home = t(i) - (traj_duration_linear + pause_duration + traj_duration_circle + pause_duration);
%trovo il numero di passi temporali necessari per completare i due tratti,
%li sommo e poi con round() estrae indice colonna p_d (ultimo elemento)
       p_final_circle = p_d(:, round((traj_duration_linear + pause_duration) / T) + round(traj_duration_circle / T));
       segment_length_home = norm(p0 - p_final_circle);

       s_pos_home = trapezoidal(0, segment_length_home, 0.06, traj_duration_home - pause_duration, t_home);
       p_d(:, i) = p_final_circle + (s_pos_home / segment_length_home) * (p0 - p_final_circle);

       % Orientamento costante
       quat_d(:, i) = Rot2Quat(R_d_current); 

   else
       p_d(:, i) = p0;
       quat_d(:, i) = Rot2Quat(R_d_current); 
   end

%% CASO 3 POSIZIONE E ORIENTAMENTO VERSO IL CENTRO DELLA TRAIETTORIA    
    case 3
        if i==1
           disp('Caso 3: Controllo con z_ee verso il centro della semicirconferenza');
        end 
    if t(i) <= traj_duration_linear

       % Tratto lineare iniziale
       s_pos = trapezoidal(0, segment_length, 0.0867, traj_duration_linear, t(i)); 
       p_d(:, i) = p0 + (s_pos / segment_length) * (p_target - p0);

       % Matrice istante iniziale tratto circolare  
       R_d = [ 0 -0 1; 0 -1 -0; 1 0 -0];  

       % Rotazione totale tra R_i e R_d
       R_rel =  R_i'*R_d;
       [asse, angle_total] = Rot2AxisAngle(R_rel);
       s_or = trapezoidal(0, angle_total, 0.13, traj_duration_linear, t(i));   
       % Rotazione progressiva
       R_d_current = R_i*Rot_axisangle(asse, s_or);  % Rotazione progressiva da R_i a R_d
       quat_d(:, i) = Rot2Quat(R_d_current);

    elseif t(i) <= traj_duration_linear + pause_duration
       % Pausa dopo il tratto lineare
       p_d(:, i) = p_d(:, i - 1);  
       quat_d(:, i) = quat_d(:, i - 1);  

    elseif t(i) <= traj_duration_linear + pause_duration + traj_duration_circle
       % Semicirconferenza
       t_circle = t(i) - (traj_duration_linear + pause_duration);
       s_pos = trapezoidal(0, pi*ro, 0.03, traj_duration_circle -pause_duration, t_circle);
       p_d(:, i) = c + R * [-ro * cos(s_pos / ro); 0; ro * sin(s_pos / ro)];

      % Vettore verso il centro
       v_centro = (c - T0(1:3,4,n));
       v_centro = v_centro / norm(v_centro);

    % Matrice di rotazione desiderata
    z_des = v_centro;                     % z deve puntare verso il centro
    x_des = -cross([0;1;0], z_des);       % x perpendicolare a z 
    x_des = x_des / norm(x_des);          % Normalizza
    y_des = cross(z_des, x_des);          % y completa l'ortogonalità
    R_d = [x_des, y_des, z_des];          % Matrice di rotazione (trasformazione dalla base alla terna desiderata)
    quat_d(:, i) = Rot2Quat(R_d); 

    elseif t(i) <= traj_duration_linear + pause_duration + traj_duration_circle + pause_duration
        % Pausa dopo la semicirconferenza
        p_d(:, i) = p_d(:, i - 1);  
        quat_d(:, i) = quat_d(:, i - 1); 

    elseif t(i) <= traj_duration_linear + pause_duration + traj_duration_circle + pause_duration + traj_duration_home
            % Ritorno alla posizione home
            t_home = t(i) - (traj_duration_linear + pause_duration + traj_duration_circle + pause_duration);
            p_final_circle = p_d(:, traj_duration_linear / T + traj_duration_circle / T);
            segment_length_home = norm(p0 - p_final_circle);

            s_pos_home = trapezoidal(0, segment_length_home, 0.06, traj_duration_home-pause_duration, t_home);
            p_d(:, i) = p_final_circle + (s_pos_home / segment_length_home) * (p0 - p_final_circle);

            quat_d(:, i) = Rot2Quat(R_d);            
    else
       p_d(:, i) = p0;
       quat_d(:, i) = Rot2Quat(R_d);
    end

%% CASO 4: OTTIMIZZAZIONE CON RIDONDANZA  
case 4  
    if i == 1
        disp('Caso 4: Ottimizzazione con ridondanza');
    end         

    if t(i) <= traj_duration_linear
        % Tratto lineare iniziale
        s_pos = trapezoidal(0, segment_length, 0.0867, traj_duration_linear, t(i)); 
        p_d(:, i) = p0 + (s_pos / segment_length) * (p_target - p0);

        % Direzione finale
        % direzione_finale = c - p_target; 
        % angolo_finale = acos(dot(z_in, direzione_finale) / norm(direzione_finale));  % Angolo tra asse z e direzione finale
        % 
        % % Rotazione progressiva tramite un profilo trapezoidale
        % s_or = trapezoidal(0, angolo_finale, 0.1153, traj_duration_linear, t(i));  % Rotazione graduale
        % R_d_current = Rot_axisangle(cross(z_in, direzione_finale), s_or);
        % 
        % quat_d(:, i) = Rot2Quat(R_d_current * R_i);

        % estraggo matrice istante iniziale tratto circolare  
        R_d = [ 0 -0 1; 0 -1 -0; 1 0 -0];  

        % Calcolo della rotazione totale tra R_i e R_d
        R_rel = R_i'*R_d;
        [asse, angle_total] = Rot2AxisAngle(R_rel);  % Asse e angolo totale
        s_or = trapezoidal(0, angle_total, 0.13, traj_duration_linear, t(i)); 
        R_d_current = R_i*Rot_axisangle(asse, s_or);  % Rotazione progressiva da R_i a R_d
        quat_d(:, i) = Rot2Quat(R_d_current);

    elseif t(i) <= traj_duration_linear + pause_duration
        % Pausa dopo il tratto lineare
        p_d(:, i) = p_d(:, i - 1);  
        quat_d(:, i) = quat_d(:, i - 1);  

    elseif t(i) <= traj_duration_linear + pause_duration + traj_duration_circle
        % Semicirconferenza
        t_circle = t(i) - (traj_duration_linear + pause_duration);
        s_pos = trapezoidal(0, pi * ro, 0.04, traj_duration_circle- pause_duration, t_circle);
        p_d(:, i) = c + R * [-ro * cos(s_pos / ro); 0; ro * sin(s_pos / ro)];

        % Vettore che punta verso il centro
        v_centro = (c - T0(1:3, 4, n));
        v_centro = v_centro / norm(v_centro);  % Normalizzazione
        z_des = v_centro;                      % z deve puntare verso il centro
        x_des = -cross([0; 1; 0], z_des);      % x perpendicolare a z
        x_des = x_des / norm(x_des);           % Normalizza
        y_des = cross(z_des, x_des);           % y completa l'ortogonalità
        R_d = [x_des, y_des, z_des];           % Matrice di rotazione
 
        quat_d(:, i) = Rot2Quat(R_d);        

    elseif t(i) <= traj_duration_linear + pause_duration + traj_duration_circle + pause_duration
        % Pausa dopo la semicirconferenza
        p_d(:, i) = p_d(:, i - 1);  
        quat_d(:, i) = quat_d(:, i - 1);  

    elseif t(i) <= traj_duration_linear + pause_duration + traj_duration_circle + pause_duration + traj_duration_home
        % Fase di ritorno a home 
        t_home = t(i) - (traj_duration_linear + pause_duration + traj_duration_circle + pause_duration);

        if temp == 0
            q_start = q(:, i-1);  % Configurazione di partenza per il rientro
            temp = 1;
        end

        [q_d(:, i), dq_d(:, i), ~] = trapezoidal(q_start, q_home, dq_c, traj_duration_home -2* pause_duration, t_home);
        
        p_d(:, i) = T0(1:3, 4, n);     
        quat_d(:, i) = Rot2Quat(T0(1:3,1:3,n));
    else
        p_d(:, i) = T0(1:3, 4, n);
        quat_d(:, i) = Rot2Quat(T0(1:3,1:3,n));
    end    
end
    %% Cinematica diretta per posizione corrente
    DH(:, 4) = q(:, i);
    T0 = DirectKinematics(DH);
    p(:, i) = T0(1:3, 4, n); 

    quat(:, i) = Rot2Quat(T0(1:3, 1:3, n));

    %% Calcolo Jacobiano
    J = Jacobian(DH);
    cond_J(i) = cond(J);

    %% Calcolo errore   
    % Inverse kinematics algorithm
    error_pos(:, i) = p_d(:, i) - p(:, i);
    error_quat(:,i) = QuatError(quat_d(:,i),quat(:,i));
    error(:,i) = [error_pos(:,i);error_quat(:,i)];

%% Calcolo velocità angolare dei giunti con saturazione
if FLAG.caso == 4
if t(i) < traj_duration_linear + traj_duration_circle
    if strcmp(opzione, 'trasposta')
        dq(:, i) = J' * K * error(:, i);
    else
        dq(:, i) = pinv_dumped(J, eye(n)) * K * error(:, i);
    end
elseif t(i) < traj_duration_linear + traj_duration_circle + traj_duration_home

    dq(:, i) = dq_d(:, i);
    
end
% Saturazione limiti di giunto 
dq(:, i) = dq(:, i); 
dq(:, i) = min(dq(:, i), joint_upper_limit); % Limite massimo
dq(:, i) = max(dq(:, i), joint_lower_limit); % Limite minimo

else
    %% Calcolo velocità angolare dei giunti con saturazione
    %% CASO 1 2 3: Calcolo generico
    if strcmp(opzione, 'trasposta')
        dq(:, i) = J' * K * error(:, i);
    else        
        dq(:, i) = pinv(J) * K * error(:, i);        
    end
    
end 

    %% Integrazione per posizione angolare
    if i < N
        q(:, i + 1) = q(:, i) + T * dq(:, i);
    end

    %% Visualizzazione robot
    if all([strcmp(FLAG.robot,'gen3'); mod(t(i),.2)==0]) %.1
        q_Gen = mask_q_DH2Gen(q(:,i)); 
        show(robot,q_Gen,'PreservePlot',true,'Frames','off');
        hold on
        plot3(p_d(1,1:i),p_d(2,1:i),p_d(3,1:i),'r', 'LineWidth', 2)

        % Disegna il centro della circonferenza
        plot3(c(1), c(2), c(3), 'bo', 'MarkerSize', 8, 'LineWidth', 2); % Punto per il centro

        % Disegna la linea tra l'end-effector e il centro
        plot3([p(1, i), c(1)], [p(2, i), c(2)], [p(3, i), c(3)], 'g', 'LineWidth', 1.5); % Li

        axis([-.4 .4 -.4 .4 -.1 1])
        view(45,35)
        drawnow
        hold off
    end
end   

%% Plot dei risultati
figure
subplot(231)
plot(t, q, '-', 'LineWidth', 1), grid on
ylabel('q [rad]')
xlabel('Tempo [s]')
xlim([0 tf]) % Limite dell'asse temporale
title('Posizione giunti')

subplot(234)
plot(t, dq, '-', 'LineWidth', 1), grid on
ylabel('dq [rad/s]')
xlabel('Tempo [s]')
xlim([0 tf])
title('Velocità angolare giunti')
legend

subplot(232)
plot(t, error_pos, '-', 'LineWidth', 1), grid on
ylabel('p err [m]')
xlabel('Tempo [s]')
xlim([0 tf])
title('Errore posizione')

subplot(235)
plot(t, error_quat, '-', 'LineWidth', 1), grid on
ylabel('o err [-]')
xlim([0 tf])
title('Errore orientamento (quaternione)')
xlabel('Tempo [s]')

subplot(233)
plot(t, cond_J, '-', 'LineWidth', 1), grid on
ylabel('cond J')
xlabel('Tempo [s]')
xlim([0 tf])
title('Condizionamento Jacobiano')

subplot(236)
plot(t,p_d,'-','LineWidth',1), grid on
ylabel('p_d[m]')
xlim([0 tf])
title('Pos Desiderata')
xlabel('Tempo [s]')

%% Visualizzazione traiettoria
figure
plot3(p_d(1, :), p_d(2, :), p_d(3, :), 'r-', 'LineWidth', 2)
grid on
axis equal
xlabel('X [m]')
ylabel('Y [m]')
zlabel('Z [m]')
title('Traiettoria desiderata del robot')
