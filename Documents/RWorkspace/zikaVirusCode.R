zikaData <- read.csv("D:/Nauka/R/Zika Virus kaggle project/cdc_zika.csv")
nrow(zikaData)

  #Checking data quality
anyDuplicated(zikaData)
duplicates <- zikaData[duplicated(zikaData), ]
head(duplicates) #Looks like there are some duplicates in the data
nrow(duplicates)
zikaDataNoDupl <- zikaData[!duplicated(zikaData), ] #Duplicates removed
nrow(zikaDataNoDupl)
#Just checking duplicated rows
# library(sqldf)
# tempData <- sqldf("select * from zikaData where location = 'El_Salvador-San_Vicente' AND data_field_code = 'SV0011' ")
#It looks like there are some rows where location names are not consistent
#I think it would be good to change all the names to lower cases

#Let's look at Argentina first 
zikaDataFilter <- zikaDataNoDupl[ grep("^Argentina", zikaData$location), ]
nrow(zikaDataFilter)
table( factor( zikaDataFilter$location_type))
table( factor( zikaDataFilter$unit)) #Check if all the observations are marked as cases
table( factor( zikaDataFilter$report_date))
View(zikaDataFilter)

#Since all the data are measured as cases, we can create a table for values from data_field will be variables
#We can use dcast for this purpose
library(reshape2)
tempDataWide <- zikaDataFilter[ ,c(2,4,8) ]
tempDataWide[ ,3] <- as.numeric(tempDataWide[ ,3]) 
zikaFilterWide <- dcast(tempDataWide, tolower(location) ~ data_field, sum)
View(zikaFilterWide)

#Correlation matrix
View(cor(zikaFilterWide[,c(2:7)], method = "pearson"))

#Just checking, how the linear regression model will fit, which variables will be the most important in that case
model <- lm(cumulative_confirmed_local_cases ~ . + 0,data=zikaFilterWide[,c(5,6,7)])
summary(model)
model <- lm(cumulative_confirmed_local_cases ~ . + 0,data=zikaFilterWide[,c(2:7)])
summary(model)