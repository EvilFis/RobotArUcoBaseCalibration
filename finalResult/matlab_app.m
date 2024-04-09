for i=1:50
    detail(i) = "./workpiece_models/plate" + num2str(i) + ".stl";
end

config = yamlread("./config.yaml");

% d_num=8;

z_pos = config.detection_parameters.robot_position_xyz(3);

% p1 = [372; -75; z_pos];
% p2 = [879; 102; z_pos]; 

p2 = [x1; y1; z_pos];  % нижняя левая точка детали
p1 = [x2; y2; z_pos];   % верхняя левая точка

data = detail2roboticCP(detail(d_num), p1, p2, ... 
                        config.detection_parameters.l, ...
                        config.detection_parameters.rTorch);

format short
positions = [data.Points data.Orientation];

match = wildcardPattern + "/";
file_name = erase(erase(detail(d_num), match), ".stl");

generate_kuka_program(["./robot_prog/", file_name, ".src"], positions);

