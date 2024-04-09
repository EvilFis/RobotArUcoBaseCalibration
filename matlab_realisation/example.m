detail = "plate43.stl";

w  = 5;                 % толщина            
p1 = [690; -151; 162];  % нижняя левая точка детали
p2 = [465; 377; 162];   % верхняя левая точка
l  = 10;                % длина факела
rTorch = 50;            % интервал между траекторями

data = detail2roboticCP (detail, p1, p2, l, rTorch);

format short
positions = [data.Points data.Orientation];


file_name = ["kuka_program2" ".src"];

generate_kuka_program(file_name, positions);