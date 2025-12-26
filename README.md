[装甲板识别参考](https://blog.csdn.net/u010750137/article/details/96428059)
[灯条识别参考](https://github.com/SEU-SuperNova-CVRA/Robomaster2018-SEU-OpenSource/tree/master/Armor)
# 识别参数
大装甲板resize：200，阈值0.75（这个能精确识别，如果出现误识别那就是你的问题）
大装甲板1.jpg是一个正方形像素，模板是正方形会更好，因为resize是正方形，你也可以不resize自己试一试。其他模板没有更新是因为懒
更新发布tf后不再更新，为了模块化，将2个节点分为2个包