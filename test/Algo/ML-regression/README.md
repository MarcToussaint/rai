Examples from the ML lexture:

```
# regression: linear features on linear data
./x.exe -mode 1 -dataFeatureType 1 -modelFeatureType 1


# regression: RBF features on sinus data
./x.exe -mode 1 -n 40 -modelFeatureType 4 -dataType 2 -rbfWidth .1 -sigma .5 -lambda 1e-10

# cross validation: quadratic features on sinus data
./x.exe -mode 4 -n 10 -modelFeatureType 2 -dataType 2 -sigma .1

./x.exe -mode 1 -n 10 -modelFeatureType 2 -dataType 2 -sigma .1


# binary classification: cubic on cluster data
./x.exe -mode 2 -d 2 -n 200 -modelFeatureType 3 -lambda 1e+0

# binary classification: RBF on cluster data
./x.exe -mode 2 -d 2 -n 200 -modelFeatureType 4 -lambda 1e+0 -rbfBias 0 -rbfWidth .2

# M=3 classification: cubic on cluster data
./x.exe -mode 3 -d 2 -n 200 -modelFeatureType 3 -lambda 1e+1

# M=3 classification: RBF on cluster data
./x.exe -mode 3 -d 2 -n 100 -modelFeatureType 4 -lambda 1e+1 -rbfBias 0 -rbfWidth .5
```

