classdef BackwardReachableChain
    % Backward reachable sets
    
    properties
        activation_pattern
        nodes
        ndim
    end
    
    methods
        function obj = BackwardReachableChain(activation_pattern, nodes)
            % Chain of backward reachable nodes
            % activation_pattern: (a1, a2, a3, .., an)
            % nodes: (n1, n2, n3, ..., nn+1)
            % n1 = X_goal, n_{i+1} -> n_{i} was followed from a_i dynamics at
            % the forward pass
            obj.activation_pattern = activation_pattern;
            obj.nodes = nodes;

            assert(length(nodes) == length(activation_pattern) + 1, ...
                "the activation pattern length should be the length of the nodes");

            obj.ndim = obj.nodes(end).Dim;
        end

        % Custom size method
        function s = size(obj, varargin)
            % Returns (1 by n_nodes)
            s = length(obj.nodes);
        end

        function union_node = union(obj)
            union_node = PolyUnion(obj.nodes);
        end

        % Custom subsref method to define subscripting behavior
        % . -- if no argument, get property, if one or more argument, get
        %      function
        % () -- slicing
        function result = subsref(obj, S, varargin)
            if isequal(S(1).type, '.')
                % Handle property access
                if length(S) == 1
                    result = obj.(S(1).subs);
                else
                    result = builtin('subsref',obj, S);
                end
            elseif isequal(S(1).type, '()')
                % Handle custom subscripting behavior
                indices = S(1).subs;
                node    = obj.nodes(indices{:});
                ap      = obj.activation_pattern(indices{:}(1:end-1));
                result  = BackwardReachableChain(ap, node);
            else
                error('Unsupported subscripting');
            end
        end

%         function obj = subsasgn(obj, S, value)
%             error("Not Implemented.")
%         end
% 
%         % Custom numArgumentsFromSubscripts method
%         function n = numArgumentsFromSubscripts(obj, s, indexingContext)
%             n = numel(s);
%         end

        function projected_chain = projection(obj, dim)
            % dim: vector of projection index
            if max(dim) > obj.ndim
                error("The maximum of projection dimension should not be " + ...
                    "larger than original dimension.")
            end
            projected_nodes = [];
            for i = 1:obj.size
                node_i = obj.nodes(i).projection(dim);
                projected_nodes = [projected_nodes, node_i];
            end

            projected_chain = BackwardReachableChain(obj.activation_pattern, projected_nodes);
        end
    end
end

