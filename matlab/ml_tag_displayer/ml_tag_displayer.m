function ml_tag_displayer(NUM_TAGS)

    phase_bits = zeros(6, 5, 3);

    % phase signature pattern
    phase_bits(1, 1, 1) = 1;
    phase_bits(2, 1, 1) = 0;
    phase_bits(3, 1, 1) = 0;
    phase_bits(4, 1, 1) = 0;
    phase_bits(5, 1, 1) = 1;
    phase_bits(6, 1, 1) = 1;

    decimals = zeros(6, 5);
    binvector = zeros(1,3);
    
    for tag_index = 1:NUM_TAGS
    
        for i = 1:6
            for j = 1:5
                for k = 1:3
                    if ~(j==1 && k==1)
                        phase_bits(i, j, k) = randi([0,1]);
                    end
                    binvector(4-k) = phase_bits(i,j,k);
                end
                decimals(i,j) = bi2de(binvector);
            end
        end

        [tag, phases] = genFTag2Marker6S5F3B(phase_bits, 1200);
        %imshow(tag);
        
        for i=1:6
            for j=1:5
                binvector(3) = phase_bits(i,j,1);
                binvector(2) = phase_bits(i,j,2);
                binvector(1) = phase_bits(i,j,3);
                decimals(i,j) = bi2de(binvector);
            end
        end
        decimals;
        phases;
                    
        file_name = sprintf('images/ftag2_6S5F3B_');
        for i=1:6
            for j=1:5
                file_name = sprintf('%s%i',file_name,decimals(i,j));
            end
            if i~=6
                file_name = sprintf('%s_',file_name);
            end
        end
        file_name = sprintf('%s.png',file_name);
        imwrite(tag,file_name);
    end

end