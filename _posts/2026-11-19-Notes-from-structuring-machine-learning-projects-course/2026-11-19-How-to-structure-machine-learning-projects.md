---
layout: post
title: How to structure machine learning projects?
category: machine learning
featured: true
mathjax: true
---

Recently I have been helping some colleagues with model training. After the recent developments in AI agents, I could iterate faster and faster on projects. But to keep the fast iteration meaningful, it is important to understand what changes are logical to achieve better score on the chosen metric. 

It is important to understand 
* How to do changes while improving your models, you have to make sure that you can clearly see the effects of the changes you did and judge in the end if it was a good idea or not
* What to do if there are multiple criteria to optimize over during training 
* If the train/dev/test sets should come from the same distribution or not
* What is Bayes error and avoidable bias
* What are the differences between high bias, high variance and data mismatch problems
* Does it make sense to use end to end learning all the time

### Orthogonality

There are so many hyperparameters to tune. 
Orthogonalization: which parameters to tune in order to achieve **one** effect.

So if you have multiple knobs to adjust the quality of the system, either tune one at a time, or if you want to tune multiple knobs, make sure that the effects of these two knobs are orthogonal to each other. This way it is possible to differentiate the effects of changing one knob from another.

### Single Evaluation Metric

While iterating through so many different models, it is important to pick a single evaluation metric depending on the project requirements.

Using only accuracy in case of imbalanced data might give a very high score but in fact the model would be bad. For example in case of a segmentation background covers most of the area, and if the model somehow predicts the whole image as background without assigning any foreground labels, then the network probably would still have above 95% accuracy. 

There might be cases that you only care that you don't miss any cases, so as few false negatives as possible and you don't care if some background parts are labelled as foreground. Then you can pick recall.

If you do not want to give false alarms, then it is better to use precision. The more precise a model is the lower its false positive rate.

If precision and recall are equally important for you, F1 score makes sense, since it is the harmonic mean of the two.

$$
\text{F1} = \frac{2\,TP}{2\,TP + FP + FN}
$$

Or if you want to weight the importance of false positives and false negatives an alternative is the Tversky score. It then adds two more hyperparameters α and β:

$$
\text{Tversky} = \frac{TP}{TP + \alpha \cdot FP + \beta \cdot FN}
$$

There are many other metrics to be used, the most important thing is to pick one metric before starting to build your system.

### Satisficing and Optimizing Metric

Even though having a single evaluation metric is desired, it is not always easy finding single evaluation metric. Often it is necessary to optimize over multiple criteria. For example, you want a high accuracy model while keeping the execution time under 50 ms per sample. 

$$
\begin{aligned}
\text{maximize} \quad & \text{accuracy} \\
\text{subject to} \quad & \text{execution time} \leq 50 \text{ ms}
\end{aligned}
$$

So in this case the accuracy corresponds to the optimizing metric, while execution time is a satisficing metric. Satisficing metric means that it just has to be good enough, so in this case if the execution time of a model is less than 50 ms, it is good enough, we do not care how low it is.

### How to set up your dev and test set

Assume you have a problem at hand that you want to train a tiny network for it, the dev and test set represents the target distribution you want to perform well on. They represent the data distribution that this network will use in the real world. 

If you build a cat classifier and your dev+test data consist of only high resolution images from the internet, then the network even though it scores well on the evaluation metric, will not be able to generalize to real world usage, assuming that it is using low quality images from users phones. 

Doing well on your metric + dev/test set does not correspond to doing well on the actual application. It is important to pick dev and test set from the same distribution. The train set can come from a slightly different distribution.

Quickly create an evaluation metric + dev/test set, even if it does not target your goal correctly you can iterate and change your metric or dev/test set distribution. This way you develop more efficiently.

### Bayes Error, Avoidable Bias, and Human level performance

Bayes error is the lowest possible error a model can achieve under a known distribution. It is usually approximated using the human level performance.

Assume that you are trying to calculate the human level error for medical image classification

|                 | Error |
|-----------------|-------|
| **Typical human error** | 3% |
| **Typical doctor error** | 1% |
| **Experienced doctor error**   | 0.7% |
| **Team of Experienced doctors error**   | 0.5% |

By looking at the above table we can take team of experienced doctors error rate as the human level performance. This means 

$$
\text{Bayes error} \leq 0.5\%
$$

#### Avoidable Bias, high bias

|                 | Error |
|-----------------|-------|
| **Human error** | 1% |
| **Train error** | 8% |
| **Dev error**   | 10% |

The difference between the human error and the train error represents avoidable bias. High bias here means that you can pick a larger network to train because your network is underfitting the data, and cannot learn all the aspects. Or train longer, or use better optimization algorithm. You can also try hyperparameter search to find more suitable NN architecture.

#### High Variance

|                 | Error |
|-----------------|-------|
| **Human error** | 1% |
| **Train error** | 1.5% |
| **Dev error**   | 10% |

The difference between the train and dev error shows that you have high variance issue. Meaning that you are maybe overfitting to your training set and you cannot generalize to the dev set, you can try adding regularization, L2 regularization, dropout, batch normalization, to prevent overfitting. You can also train on more data to generalize better. You can also try hyperparameter search to find more suitable NN architecture.


### Error Analysis

Very important!!!!

If your network does not give good score on your metric, it could be smart to do an error analysis, so that you can decide which step is more logical to take. So you can be more efficient and also gain more.

Assume you have a cat classifier and you realized you label some dog images as cats. Best thing to do is to get 100 mislabeled images (depending on the size of your dataset you can take more or less) then check how many of these images are mislabeled as dog. Depending on the ratio you can check how much gain fixing this would give you. Then if it is a good drop in the error (eg. error going from 10% to 5%) then it might be worth your time to collect more dog images and label them and train the network again. 

By doing error analysis, you can easily decide which step to take next, and iterate quickly rather than spending so much time on something and then it giving not enough performance boost.

#### Cleaning up incorrectly labeled data

If the incorrectly labeled data is only in the train set and is not systematic, then it is okay to not clean this.
But it is important to have correctly labeled data in your dev and test set, since these datasets are used to evaluate your model.
If you end up cleaning dev set, you should also clean the test set. Otherwise they will come from different data distributions. So the most important thing here is if you clean dev set and go ahead and do the same thing for the test set, and vice versa.


### References
* [Structuring Machine Learning Projects course on Coursera](https://coursera.org/share/5865fd7fe725a02811fb6797d1101193)
