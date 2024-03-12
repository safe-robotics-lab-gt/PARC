function isHullNeeded = check_vertex_intersection(P_1, P_2, k_v_lo, k_v_hi, k_a_lo, k_a_hi, k_pk_lo, k_pk_hi)
%CHECK_VERTEX_INTERSECTION Summary of this function goes here
%   Detailed explanation goes here
isHullNeeded = false;

for i = [k_v_lo, k_v_hi]
    for j = [k_a_lo, k_a_hi]
        for k = [k_pk_lo, k_pk_hi]
            P_1_y = P_1.slice(2:4, [i;j;k]);
            P_1_y.minHRep();
            assert(length(P_1_y.A) == 2)
            if P_1_y.A(1) == 1
                assert(P_1_y.A(2) == -1)
                P_1_y_ub = P_1_y.b(1);
                P_1_y_lb = -P_1_y.b(2);
            elseif P_1_y.A(1) == -1
                assert(P_1_y.A(2) == 1)
                P_1_y_lb = -P_1_y.b(1);
                P_1_y_ub = P_1_y.b(2);
            else
                error("Unexpected interval")
            end

            P_2_y = P_2.slice(2:4, [i;j;k]);
            P_2_y.minHRep();
            assert(length(P_2_y.A) == 2)
            if P_2_y.A(1) == 1
                assert(P_2_y.A(2) == -1)
                P_2_y_ub = P_2_y.b(1);
                P_2_y_lb = -P_2_y.b(2);
            elseif P_2_y.A(1) == -1
                assert(P_2_y.A(2) == 1)
                P_2_y_lb = -P_2_y.b(1);
                P_2_y_ub = P_2_y.b(2);
            else
                error("Unexpected interval")
            end

            if P_1_y_lb > P_2_y_ub || P_2_y_lb > P_1_y_ub
                isHullNeeded = true;
                return
            end
        end
    end
end

end

