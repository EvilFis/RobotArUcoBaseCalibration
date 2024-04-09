function generate_kuka_program(file_name, positions)

    % шапка
    cap = join (['DEF ' file_name(1) '()\n'],"");

    cap2 = ["GLOBAL INTERRUPT DECL 3 WHEN $STOPMESS==TRUE DO IR_STOPM ( )";
    "INTERRUPT ON 3";
    "BAS (#INITMOV,0)";
    "BAS (#VEL_PTP,20)";
    "BAS (#ACC_PTP,20)";
    "$BASE=BASE_DATA[1]";
    "$TOOL=TOOL_DATA[16]"
    "$VEL.CP=2"
    "$ACC.CP=2"
    ""
    "; Home"
    "PTP {A1 0, A2 -90, A3 90, A4 0, A5 0, A6 0, E1 0} C_DIS"    ];

    name = join(file_name,"");
    fid = fopen(name, 'w');
    
    % заполнение шапки
    fprintf(fid, cap);
    for i = 1 : size(cap2,1)
    fprintf(fid, cap2(i));
    fprintf(fid, '\n');
    end
    
    % команды для траектории
    for i = 1:size(positions, 1)
        if i == 1
            fprintf(fid, 'PTP {X %.3f, Y %.3f, Z %.3f, A %.3f, B %.3f, C %.3f} C_DIS\n', positions(i,:));
        else
            fprintf(fid, 'LIN {X %.3f, Y %.3f, Z %.3f, A %.3f, B %.3f, C %.3f} C_DIS\n', positions(i,:));
        end
    end
    
    fprintf(fid, '\n');
    fprintf(fid, 'END');
    
    fclose(fid);
    
    disp(['Generated KUKA program saved in ' name]);
    
end



