function confmat = cnbiproc_confusionmat(labels, predictions, varargin)
% confmat = cnbiproc_confusionmat(labels, prediction [, 'classes', ClassValues, 'rejection', RejValue])
%
% The function computes the confusion matrix given a vector of labels
% and predictions. Predictions can be a vector with the same length of 
% labels or a matrix with posterior probabilities with same length of
% labels. The function returns a confusion matrix
% (real classe per rows, predicted class per columns) with an additional
% column at the end with the percentage of rejected samples. Note: 
% the sub square matrix (standard confusion matrix) is normalized with
% respect to the non-rejected samples (sum of row to 100). The last column
% of the confusion matrix (rejected sampels) is normalized with respect the
% total number of samples.
%
% Input:
%   - labels                    Numeric vector with real labels [samples x 1]
%   - preditictions             Numeric vector with guess labels [samples x 1] or
%                               probabilities matrix [samples x nclasses]
%   - 'classes', ClassValues    Optional argument: class labels (values and
%                               order is considered) 
%                               [Default: class values in labels argument, in ascending order]
%   - 'rejection', RejValue     Optional argument: rejection value 
%                               [Default: 1/nclasses]
%
% SEE ALSO: confusionmat

    % Check labels and predictions length
    if isequal(size(labels, 1), size(predictions, 1)) == false
        error('chk:lb', 'Labels and predictions must have the same number of samples');
    end
    
    % Getting general parameter from labels argument
    nsamples = size(labels, 1);
    classes = unique(labels)';
    nclasses = length(classes);
    
    % Parsing optional input arguments
    p = inputParser;
    p.PartialMatching = 0;

    paramClass    = 'classes';
    defaultClass  = classes;
    paramClassErr = ['Classes label must be numeric and of length ' num2str(nclasses)];
    paramClassFcn = @(x) assert(isnumeric(x) && (length(x) == nclasses), paramClassErr);
    addParameter(p, paramClass, defaultClass, paramClassFcn);
    
    paramRej    = 'rejection';
    defaultRej  = 1/nclasses;
    paramRejErr = 'Rejection must be a number';
    paramRejFcn = @(x) assert(isnumeric(x) && (length(x) == 1), paramRejErr);
    addParameter(p, paramRej, defaultRej, paramRejFcn);

    parse(p, varargin{:});
    
    reqclasses = p.Results.classes;
    rejection  = p.Results.rejection;
    
    % Converting probabilities into predicted labels and checking for
    % rejected values
    rejlabels = false(nsamples, 1);
    if isvector(predictions) == false
        [maxprobs, tmplb] = max(predictions, [], 2); 
        guesslabels = tmplb;
        for cId = 1:nclasses
            guesslabels(guesslabels == cId) = reqclasses(cId);
        end
        
        rejlabels = maxprobs < rejection;
    else
        guesslabels = predictions;
    end
    
    % Computing number of rejected samples per class
    rj = zeros(nclasses, 1);
    for cId = 1:nclasses
        rj(cId) = sum(rejlabels(labels == classes(cId)));
    end
    
    
    % Checking if the values inside labels and predictions are the same
    if isequal(unique(labels), unique(guesslabels)) == false
        
        warning('chk:lb', 'Real labels and guess labels have different class values')
    end
    
    % Compute the confusion matrix per class
    cm  = confusionmat(labels(rejlabels == false), guesslabels(rejlabels == false));
    
    % Computing percentage (with respect to the non-rejected samples for
    % the standard confusion matrix, with respect to the overall number of
    % samples for the rejection column)
    pcm = 100*cm./repmat(sum(cm, 2), [1 size(cm, 2)]);
    prj = 100*rj./sum([cm rj], 2);
    
    % Adding rejection column at the end
    confmat = cat(2, pcm, prj);
    

end