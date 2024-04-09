function [trajData] = detail2roboticCP (detail, p1, p2, l, rTorch)

trajData = struct("Points", [], "Orientation", []);

gm2 = fegeometry(detail);
triang = stlread(detail);

vert = gm2.Vertices;
nodes = triang.Points;

n = size (nodes,1);
m = size (nodes,2);

Tinstr3 = [ 1   0  0  0;
            0   -1 0  0;
            0   0  -1 0;
            0   0  0  1];

Tinstr2 = [ 0   0  1  0;
            0   -1  0  0;
            1   0  0  0;
            0   0  0  1];

Tinstr1 = [ 0   -1  0  0;
            1   0   0  0;
            0   0   1  0;
            0   0   0  1];
Tinstr = Tinstr1*Tinstr2*Tinstr3;

% Tinstr2 = [  1   0  0  0;
%             0   1  0  0;
%             0   0  1  0;
%             0   0  0  1];

%% Сортировка вершин полигонов

v = sorting(n, nodes);

%% Перенос системы координат в нижнюю левую точку

gm2 = translate(gm2,[-v(1,1) -v(1,2) -v(1,3)]);
% Можно повернуть на угол относительно оси Х [1 0 0], Y[0 1 0]
%theta = -90;
%gm2 = rotate (gm2,theta,[0 0 0], [1 0 0]);

triang = triangulation(gm2);
nodes = triang.Points;

%% Повторная сортировка после переноса с.к.

v = sorting(n, nodes);

%% Формирование точек покраски (Перпендикулярные к поверхности)

n = size(v,1);
for i = 1 : n-1

    d = v(i,:);
    d(1) = d(1) + 1;

    p(i,:) = norm2Surf(v(i+1,:),v(i,:),d,v(i,:),l);

end
p(i+1,:) = norm2Surf(v(i+1,:),v(i,:),d,v(i+1,:),l);

%% Формирование ориентации инструмента для точек (матрица поворотов)

for i = 1:n

    z_p(1,1:3) = p(i,1:3);
    z_p(2,1:3) = v(i,1:3);

    pZ = (z_p(2,1:3) - z_p(1,1:3));
    pZr = (pZ/norm(pZ))';
    pYr = [1; 0; 0];
    pXr = -cross(pZr,pYr);
    % pXr = -[1; 0; 0];
    % pYr = cross(pZr,pXr);

    rotMatrices(1:3,1:3,i) =[pXr pYr pZr] * Tinstr(1:3,1:3);
    %rotMatrices(1:3,1:3,i) =[pXr pYr pZr];


end

%Отобразить деталь
% pdegplot(gm2);


%отобразить траекторию локально к заготовке
% plot3(p(:,1), p(:,2), p(:,3), Color = '#02a5ff', LineWidth=1.2);
% axis equal
% hold on
% plot3(v(:,1), v(:,2), v(:,3), Color = '#66cc33', LineWidth=1.8);

% for i = 1:4:n
% plotCoordinateFrame(p(i,1),p(i,2),p(i,3),rotMatrices(:,:,i),l/2);
% end

%% Формирование траектории покраски (точки и ориентация для каждой точки)

% Вычисляется длина заготовки и количество "прогонов" покрашивания в
% зависимости от интервала между ними rTorch
length = max(vert(:,1)) - min(vert(:,1));
tryes = ceil (length/rTorch);

[traj, orientTraj] = trajCreator(rTorch, tryes, p, rotMatrices);

% plot3(traj(:,1), traj(:,2), traj(:,3),Color = '#02a5ff', LineWidth=1.3);
% 
% k = size(traj,1);
% for i = 1:5:k
% plotCoordinateFrame(traj(i,1),traj(i,2),traj(i,3),orientTraj(:,:,i),l/2);
% end

%%
T = points2matrice(p1,p2);
% r = [ 1  0  0;
%       0  0  1;
%       0  -1 0];

% T(1:3,1:3) = r;

[newTraj, newOrient1] = translateTraj(T, traj, orientTraj);

%[fakeNewTraj, newOrient1] = translateTraj(inv(Tinstr2), newTraj', newOrient1);

%% Заполнение данных о траектории покраски

trajData.Points = newTraj(1:3,:)';

n = size(traj,1);
for i = 1 : n

    eul = rotm2eul(newOrient1(1:3,1:3,i));
    eul = rad2deg(eul);

    % ang = rotm2axang(newOrient2(1:3,1:3,i));
    % quat = axang2quat(ang);
    % eul = quat2eul(quat);
    % ABC = rad2deg(eul);

    % eul = rotm2eul(newOrient2(1:3,1:3,i));
    trajData.Orientation(i,1:3) = (eul);

end

%% Перенос системы координат в нижнюю левую точку

angleTr = rad2deg(rotm2eul(T(1:3,1:3)));

gm2 = translate(gm2,[T(1,4) T(2,4) T(3,4)]);


% Можно повернуть на угол относительно оси Х [1 0 0], Y[0 1 0]
theta = 90;
gm2 = rotate (gm2,theta,[T(1,4) T(2,4) T(3,4)], [T(1,4) T(2,4) T(3,4)+1]);
gm2 = rotate (gm2,-theta,[T(1,4) T(2,4) T(3,4)], [T(1,4) T(2,4)+1 T(3,4)]);
%gm2 = rotate (gm2,angleTr(3),[T(1,4) T(2,4) T(3,4)], [T(1,4)+1 T(2,4) T(3,4)]);


%% Отображение

pdegplot(gm2)
xlabel('X') 
ylabel('Y')
zlabel('Z')
hold on
showPlot3(newTraj(1:3,:));

for i = 1 :15: n

    X = trajData.Points(i,1); Y = trajData.Points(i,2);
    Z = trajData.Points(i,3);
    R = newOrient1(1:3,1:3,i);
    A = trajData.Orientation(i,1);
    B = trajData.Orientation(i,2);
    C = trajData.Orientation(i,3);

    plotCoordinateFrameEuler(X,Y,Z,A,B,C,l/2);
    %plotCoordinateFrame(X,Y,Z,R,l/2);
    hold on
end



end

%%

function [traj, orientTraj] = trajCreator(rTorch, tryes, p, rotMatrices)

% Формирование траектории покраски (точки и ориентация)

t = p;
traj3(:,:,1) = p;
traj = p;
orientTraj = rotMatrices;
R = orientTraj;

for i = 1 : tryes

    t = flip(t);
    R = flip(R,3);
    t(:,1) = t(:,1) + rTorch;

    traj3(:,:,i+1) = t; 
    traj = [traj; t];

    orientTraj = cat(3,orientTraj,R);
   
end

% for i = 1 : tryes+1
% 
%      showPlot3(traj3(:,:,i))
%      hold on
% end

% showPlot3(traj);



end

function [newTraj, newOrient] = translateTraj(T, traj, orientTraj)


Tr = eye(4,4);

n = size(traj,1);
points = traj';

for i = 1 : n
%умножение каждой точки на матрицу положения
    pT = points(:,i);
    pT(4,1) = 1; 
    PT0=T*pT;

    if i == 1
    newTraj =PT0;
    else
    newTraj = [newTraj PT0];
    end

    Tr(1:3,1:3) = orientTraj(1:3,1:3,i);
    newTr = T*Tr;
    newOrient(1:3,1:3,i) = newTr(1:3,1:3);
    
end

end

function [T] = points2matrice (p1, p2)

% a1 = [0; 0; 0];
% a2 = [1; 1; 0];

% Приводим к нужному формату данных
if size(p1,1) == 3
a1 = p1;
a2 = p2;
elseif size(p1,2) == 3
    a1 = p1';
    a2 = p2';
else
    disp('tochki kosyаchnye (points2MHT)')
end

% Преобразуем две точки в три
A = a1;
B = [a1(1); a2(2); a1(3)];
C = [a2(1); a1(2); a2(3)];

% Задаем координаты трех точек прямоугольника
% Для примера, точки A, B и C заданы в формате [x; y; z]
% A = [0; 0; 0];
% B = [0; 1; 0];
% C = [1; 0; 0];

% Создаем матрицу из заданных точек
P = [A, B, C];

% Вычисляем векторы сторон прямоугольника
u = B - A;
v = C - A;

% Находим нормализованные векторы сторон
u = u / norm(u);
v = v / norm(v);

% Находим нормаль к плоскости прямоугольника
normal = cross(u, v);
normal = normal / norm(normal);

% Создаем матрицу однородного преобразования для описание положения прямоугольника
T = eye(4);
T(1:3, 1:3) = [u, -normal, v];
T(1:3, 4) = A;

end

function plotCoordinateFrameEuler(X, Y, Z, roll, pitch, yaw, scale)
    % X, Y, Z - координаты указанной точки
    % roll, pitch, yaw - углы Эйлера (в радианах) описывающие ориентацию

    % Создаем систему координат в указанной точке
    origin = [X, Y, Z];
    % scale = 1;
    % quiver3(origin(1), origin(2), origin(3), scale, 0, 0, 'r', 'LineWidth', 1.5); % Ось X красного цвета
    % hold on;
    % quiver3(origin(1), origin(2), origin(3), 0, scale, 0, 'g', 'LineWidth', 1.5); % Ось Y зеленого цвета
    % quiver3(origin(1), origin(2), origin(3), 0, 0, scale, 'b', 'LineWidth', 1.5); % Ось Z синего цвета

    % Определяем матрицу поворота из углов Эйлера
    eul = [yaw, pitch, roll];
    eul = deg2rad(eul);
    R = eul2rotm(flip(eul));

    % Определяем векторы осей с учетом ориентации
    Rx = R(:,1); % Ось X
    Ry = R(:,2); % Ось Y
    Rz = R(:,3); % Ось Z

    % Масштабируем векторы
    scaledRx = scale * Rx';
    scaledRy = scale * Ry';
    scaledRz = scale * Rz';

    % Рисуем векторы осей с учетом ориентации
    quiver3(X, Y, Z, scaledRx(1), scaledRx(2), scaledRx(3), 'r', 'LineWidth', 1.5); % Ось X
    hold on
    quiver3(X, Y, Z, scaledRy(1), scaledRy(2), scaledRy(3), 'g', 'LineWidth', 1.5); % Ось Y
    quiver3(X, Y, Z, scaledRz(1), scaledRz(2), scaledRz(3), 'b', 'LineWidth', 1.5); % Ось Z

    % % Настройка графика
    % xlabel('X');
    % ylabel('Y');
    % zlabel('Z');
    % axis equal;
    % grid on;
end

function Q = norm2Surf(A, B, C, P, l)

% Находим уравнение плоскости, проходящей через точки A, B и C
% Нормаль к плоскости определяется как векторное произведение векторов AB и AC
normal = cross(B - A, C - A);

% Нормализуем нормальный вектор
normal = normal / norm(normal);

% Вычисляем координаты конечной точки перпендикуляра
Q = P + l * normal;

end

function [v] = sorting (n, nodes)

% Сортировка вершин (расстановка Сверху-вниз и Слева-направо По оси Х)
s = sortrows(nodes,[1 3]);
x = s(1,1);
v = s(1,:);

for i = 2 : n
    if s(i,1) == x
        v = [v; s(i,:)];
    end
end
end

function [] = showPlot3 (A)

if size(A,2) == 3
else
    A = A';
end

if size(A,2) ~= 3
    disp('error with plot3');
else
    plot3 (A(:,1),A(:,2),A(:,3));
end
end


