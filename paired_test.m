function [h, p, m] = paired_test(x,varargin)
% Paired statistical tests. 
% Run lillietest to test normality in both x and y (in x when there's only one
% input). If either is not normal, run Kruskal Wallis. If both are normal,
% run t-test

if nargin == 1
    if size(x,2) > 1
        h = zeros(1,size(x,2));
        p = zeros(1,size(x,2));
        m = cell(1,size(x,2));
        for i = 1 : size(x,2)
            tempx = x(:,i);
            tempx = tempx(isfinite(tempx));
            if lillietest(tempx)
                m{i} = 'Wilcoxon';
                [p(i), h(i)] = signrank(tempx);
            else
                m{i} = 'ttest';
                [h(i), p(i)] = ttest(tempx);
            end
        end
    else
        tempx = x(isfinite(x));
        if lillietest(tempx)
            m = 'Wilcoxon';
            [p,h] = signrank(tempx);
        else
            m = 'ttest';
            [h,p] = ttest(tempx);
        end
    end
else
    y = varargin{1};
    if size(x,2) > 1
        if size(y,2) > 1
            if size(x,2) ~= size(y,2)
                error('Number of comparisons should be the same')
                return
            else
                h = zeros(1,size(x,2));
                p = zeros(1,size(x,2));
                m = cell(1,size(x,2));
                for i = 1 : size(x,2)
                    tempx = x(:,i); tempy = y(:,i);
                    tempInd = intersect(find(isfinite(tempx)), find(isfinite(tempy)));
                    tempx = tempx(tempInd);
                    tempy = tempy(tempInd);
                    if lillietest(tempx) || lillietest(tempy)
                        m{i} = 'Wilcoxon';
                        [p(i), h(i)] = signrank(tempx,tempy);
                    else
                        m{i} = 'ttest';
                        [h(i), p(i)] = ttest(tempx,tempy);
                    end
                end
            end
        else
            h = zeros(1,size(x,2));
            p = zeros(1,size(x,2));
            m = cell(1,size(x,2));
            for i = 1 : size(x,2)
                tempx = x(:,i);
                tempInd = intersect(find(isfinite(tempx)), find(isfinite(y)));
                tempx = tempx(tempInd);
                tempy = y(tempInd);
                if lillietest(tempx) || lillietest(tempy)
                    m{i} = 'Wilcoxon';
                    [p(i), h(i)] = signrank(tempx,tempy);
                else
                    m{i} = 'ttest';
                    [h(i), p(i)] = ttest(tempx,tempy);
                end
            end
        end
    else
        if size(y,2) > 1
            h = zeros(1,size(y,2));
            p = zeros(1,size(y,2));
            m = cell(1,size(y,2));
            for i = 1 : size(y,2)
                tempy = y(:,i);
                tempInd = intersect(find(isfinite(x)), find(isfinite(tempy)));
                tempy = tempy(tempInd);
                tempx = x(tempInd);
                if lillietest(tempx) || lillietest(tempy)
                    m{i} = 'Wilcoxon';
                    [p(i), h(i)] = signrank(tempx,tempy);
                else
                    m{i} = 'ttest';
                    [h(i), p(i)] = ttest(tempx,tempy);
                end
            end
        else
            tempInd = intersect(find(isfinite(x)), find(isfinite(y)));
            tempx = x(tempInd);
            tempy = y(tempInd);
            if lillietest(tempx) || lillietest(tempy)
                m = 'Wilcoxon';
                [p, h] = signrank(tempx,tempy);
            else
                m = 'ttest';
                [h, p] = ttest(tempx,tempy);
            end
        end
    end
end