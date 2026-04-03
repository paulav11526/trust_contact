import numpy as np
import pandas as pd
from sklearn.ensemble import RandomForestClassifier

# Feature Extraction Function
def extract_features(matrix):
    # matrix shape: (2, 3)
    f1, t1_start, t1_end = matrix[0]
    f2, t2_start, t2_end = matrix[1]
    
    duration1 = t1_end - t1_start
    duration2 = t2_end - t2_start
    gap = t2_start - t1_end  # time between contacts
    force_diff = abs(f1 - f2)
    
    return [f1, f2, duration1, duration2, gap, force_diff]

# Encoding Function
def encode_label(label):
    if label == "long tap":
        return [1, 0, 0]
    elif label == "double tap":
        return [0, 1, 0]
    else:  # "tap"
        return [0, 0, 1]

# Create Sample Training Data
np.random.seed(42)
X = []
y = []

for _ in range(300):
    # Generate two contacts
    contact1 = [
        np.random.uniform(10, 100),   # force
        np.random.uniform(0, 5),      # start
        np.random.uniform(5, 10)      # end
    ]
    
    contact2 = [
        np.random.uniform(10, 100),
        np.random.uniform(10, 15),
        np.random.uniform(15, 20)
    ]
    
    matrix = np.array([contact1, contact2])
    features = extract_features(matrix)
    
    # Label rules (example logic)
    if features[2] > 3 and features[4] > 2:
        label = "long tap"
    elif features[4] < 0.5:
        label = "double tap"
    else:
        label = "continue"
    
    X.append(features)
    y.append(encode_label(label))


X = pd.DataFrame(X, columns=[
    "force1", "force2", "duration1", "duration2", "gap", "force_diff"
])

# Train Model
model = RandomForestClassifier(n_estimators=100, random_state=42)
model.fit(X, y)

# Prediction Function
def predict_movement(matrix):
    features = extract_features(matrix)
    features_df = pd.DataFrame([features], columns=X.columns)

    pred = model.predict(features_df)[0]
    print(pred)
    return pred

# Example Prediction
test_matrix = np.array([
    [80, 1.0, 5.0],   # long contact
    [75, 8.0, 9.0]
])

prediction = predict_movement(test_matrix)
print("Prediction:", prediction)