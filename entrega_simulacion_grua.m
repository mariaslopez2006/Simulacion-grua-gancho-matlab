
function simulacion_grua_libre()
    clc;
    close all;

    %% 1. PARAMETROS
    p.M = 5;       % Masa de la barra (kg)
    p.L = 4;        % Longitud de la barra (m)
    p.m = 5;        % Masa del gancho (kg)
    p.l = 3;        % Longitud de la cuerda (m)
    p.H = 8;        % Altura inicial del centro de masas (m)
    p.g = 9.81;     % Gravedad (m/s^2)

    %% 2. CONDICIONES INICIALES
    % phi: angulo de la barra respecto a la horizontal
    % theta: angulo de la cuerda respecto a la vertical hacia abajo

    phi0 = 0;
    theta0 = pi/4;

    X0 = 0;
    Y0 = p.H;

    Xd0 = 0;
    Yd0 = 0;

    phid0 = -0.3;
    thetad0 = 0.5;

    % Estado: [X,Y,phi,theta,Xd,Yd,phid,thetad]
    z0 = [X0; Y0; phi0; theta0; ...
          Xd0; Yd0; phid0; thetad0];

    %% 3. INTEGRACION
    opciones = odeset( ...
        'RelTol', 1e-8, ...
        'AbsTol', 1e-10, ...
        'MaxStep', 0.02, ...
        'Events', @(t,z) eventoContacto(t,z,p));

    [t,z] = ode45(@(t,z) dinamica(t,z,p), ...
                  [0 20], z0, opciones);

    %% 4. POSICIONES
    X = z(:,1);
    Y = z(:,2);
    phi = z(:,3);
    theta = z(:,4);

    % Extremos de la barra
    xA = X - (p.L/2)*cos(phi);
    yA = Y - (p.L/2)*sin(phi);

    xB = X + (p.L/2)*cos(phi);
    yB = Y + (p.L/2)*sin(phi);

    % Gancho
    xG = X + p.l*sin(theta);
    yG = Y - p.l*cos(theta);

    %% 5. DESPLAZAMIENTO HORIZONTAL
    desplazamientoX = X - X0;

    fprintf('Tiempo hasta el contacto: %.4f s\n', t(end));
    fprintf('Desplazamiento horizontal final: %.4f m\n', ...
            desplazamientoX(end));
    fprintf('Desplazamiento horizontal maximo: %.4f m\n', ...
            max(abs(desplazamientoX)));

    %% 6. FIGURA Y REFERENCIAS
    figure('Color','w','Name','Simulacion de grua');

    hold on;
    grid on;
    axis equal;

    % Limites fijos para mantener X=0 en el centro visual
    xlim([-6 6]);
    ylim([-0.5 p.H + p.L + 1]);

    xlabel('Posicion horizontal X (m)');
    ylabel('Altura Y (m)');
    title('Movimiento libre de la grua');

    % Suelo
    plot([-6 6], [0 0], 'k-', 'LineWidth', 2);

    % Vertical fija de referencia
    xline(0, 'k--', 'X = 0', 'LineWidth', 1.5);

    % Trayectorias
    hTrayA = plot(nan,nan,'--','LineWidth',1);
    hTrayB = plot(nan,nan,'--','LineWidth',1);
    hTrayG = plot(nan,nan,'--','LineWidth',1);
    hTrayCM = plot(nan,nan,'-','LineWidth',2);

    % Barra, cuerda, centro de masas y gancho
    hBarra = plot(nan,nan,'b-','LineWidth',5);
    hCuerda = plot(nan,nan,'k-','LineWidth',1.5);
    hCM = plot(nan,nan,'bo','MarkerFaceColor','b','MarkerSize',7);
    hGancho = plot(nan,nan,'ro','MarkerFaceColor','r','MarkerSize',9);

    legend([hTrayA,hTrayB,hTrayG,hTrayCM,hBarra,hCuerda,hCM,hGancho], ...
        {'Trayectoria extremo A', ...
         'Trayectoria extremo B', ...
         'Trayectoria gancho', ...
         'Trayectoria centro de masas', ...
         'Barra', 'Cuerda', 'Centro de masas', 'Gancho'}, ...
         'Location','eastoutside');

    %% 7. ANIMACION
    paso = max(1, round(length(t)/600));

    for k = 1:paso:length(t)

        % Trayectorias acumuladas
        set(hTrayA,'XData',xA(1:k),'YData',yA(1:k));
        set(hTrayB,'XData',xB(1:k),'YData',yB(1:k));
        set(hTrayG,'XData',xG(1:k),'YData',yG(1:k));
        set(hTrayCM,'XData',X(1:k),'YData',Y(1:k));

        % Barra
        set(hBarra,'XData',[xA(k),xB(k)], ...
                   'YData',[yA(k),yB(k)]);

        % Cuerda
        set(hCuerda,'XData',[X(k),xG(k)], ...
                    'YData',[Y(k),yG(k)]);

        % Centro de masas
        set(hCM,'XData',X(k),'YData',Y(k));

        % Gancho
        set(hGancho,'XData',xG(k),'YData',yG(k));

        title(sprintf(['Movimiento libre: t = %.2f s\n' ...
              'Desplazamiento horizontal del centro: X-X_0 = %.3f m'], ...
              t(k), desplazamientoX(k)));

        drawnow;
    end

    % Asegurar que se muestre el ultimo instante
    k = length(t);

    set(hTrayA,'XData',xA,'YData',yA);
    set(hTrayB,'XData',xB,'YData',yB);
    set(hTrayG,'XData',xG,'YData',yG);
    set(hTrayCM,'XData',X,'YData',Y);

    set(hBarra,'XData',[xA(k),xB(k)], ...
               'YData',[yA(k),yB(k)]);
    set(hCuerda,'XData',[X(k),xG(k)], ...
                'YData',[Y(k),yG(k)]);
    set(hCM,'XData',X(k),'YData',Y(k));
    set(hGancho,'XData',xG(k),'YData',yG(k));

    title(sprintf('Fin de la simulacion: desplazamiento X = %.3f m', ...
          desplazamientoX(end)));

    drawnow;
end


%% 8. ECUACIONES DIFERENCIALES
function dz = dinamica(~,z,p)

    theta = z(4);
    Yd = z(6);
    Xd = z(5);
    phid = z(7);
    thetad = z(8);

    Mt = p.M + p.m;

    % Sistema para Xdd, Ydd y thetadd
    A = [Mt, 0, p.m*p.l*cos(theta);
         0, Mt, p.m*p.l*sin(theta);
         p.m*p.l*cos(theta), p.m*p.l*sin(theta), p.m*p.l^2];

    b = [p.m*p.l*thetad^2*sin(theta);
        -Mt*p.g - p.m*p.l*thetad^2*cos(theta);
        -p.m*p.g*p.l*sin(theta)];

    acc = A\b;

    Xdd = acc(1);
    Ydd = acc(2);
    thetadd = acc(3);

    % Ecuacion de giro de la barra del documento
    phidd = 0;

    dz = [Xd;
          Yd;
          phid;
          thetad;
          Xdd;
          Ydd;
          phidd;
          thetadd];
end


%% 9. EVENTO DE CONTACTO CON EL SUELO
function [value,isterminal,direction] = eventoContacto(~,z,p)

    Y = z(2);
    phi = z(3);
    theta = z(4);

    % Alturas de los extremos de la barra
    yA = Y - (p.L/2)*sin(phi);
    yB = Y + (p.L/2)*sin(phi);

    % Altura del gancho
    yG = Y - p.l*cos(theta);

    % Primer contacto entre los puntos considerados y el suelo
    value = min([yA,yB,yG]);

    isterminal = 1;
    direction = -1;
end
