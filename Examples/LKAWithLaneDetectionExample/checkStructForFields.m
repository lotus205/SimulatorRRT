        function flag = checkStructForFields(s, expFields)
            % Returns true if the struct s has the expected fields, expFields
            
            flag = true;
            for i = 1:numel(expFields)
                if ~isfield(s,expFields{i})
                    flag = false;
                    return
                end
            end
        end
